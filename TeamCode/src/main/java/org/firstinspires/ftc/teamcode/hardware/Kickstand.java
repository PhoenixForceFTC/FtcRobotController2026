package org.firstinspires.ftc.teamcode.hardware;

//region --- Imports ---
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;
//endregion

public class Kickstand
{
    //region --- Constants ---
    //--- Motor speed for RUN_TO_POSITION mode (max power allowed)
    private static final double MOTOR_SPEED = 0.3;  // Low speed for smooth movement
    private static final double HOLDING_POWER = 0.15;  // Lower power when holding position

    //--- Encoder positions (tune these after testing)
    //--- Position 0 = starting position (encoder reset on init)
    //--- Positive or negative depends on motor direction - adjust after testing
    private static final int POSITION_DOWN = 0;       // Starting/down position
    private static final int POSITION_UP = 500;       // Up position (tune this value)

    //--- Position tolerance (how close to target before we consider it "there")
    private static final int POSITION_TOLERANCE = 10;  // Tighter tolerance for RUN_TO_POSITION
    //endregion

    //region --- Enums ---
    public enum Position
    {
        DOWN,
        UP
    }
    //endregion

    //region --- Hardware ---
    private final DcMotor _motor;
    private final Gamepad _gamepad1;
    private final Gamepad _gamepad2;
    private final Telemetry _telemetry;
    private final boolean _showInfo;
    private final int _robotVersion;
    //endregion

    //region --- State ---
    private Position _targetPosition = Position.DOWN;
    private boolean _leftBumperPressed = false;  // Debounce for gp2 left bumper
    private boolean _isMoving = false;
    
    //--- Flywheel and Intake references (to turn off when kickstand is used)
    private Flywheel _flywheel = null;
    private Intake _intake = null;
    private Camera _camera = null;
    private Lights _lights = null;
    //endregion

    //region --- Constructor ---
    public Kickstand(
            DcMotor motor,
            Gamepad gamepad1,
            Gamepad gamepad2,
            Telemetry telemetry,
            int robotVersion,
            boolean showInfo
    )
    {
        this._motor = motor;
        this._gamepad1 = gamepad1;
        this._gamepad2 = gamepad2;
        this._telemetry = telemetry;
        this._robotVersion = robotVersion;
        this._showInfo = showInfo;
    }
    //endregion

    //region --- Initialize ---
    public void initialize()
    {
        //--- Reset encoder to 0 at current position (should be DOWN)
        _motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        //--- Configure PIDF for smooth position holding
        //--- Lower P = less aggressive, less oscillation
        //--- Higher D = more damping, reduces overshoot
        //--- F is feedforward, not typically used for position
        if (_motor instanceof DcMotorEx)
        {
            DcMotorEx motorEx = (DcMotorEx) _motor;
            //--- Tune these values to reduce bobbing:
            //--- P=5 (gentle), I=0 (no integral windup), D=2 (damping), F=0
            motorEx.setPositionPIDFCoefficients(5.0);
        }
        
        //--- Set initial target position before switching to RUN_TO_POSITION
        _motor.setTargetPosition(POSITION_DOWN);
        _motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _motor.setPower(HOLDING_POWER);  // Set power - motor will hold position
        
        _targetPosition = Position.DOWN;
        _isMoving = false;
    }

    //--- Set flywheel reference (call after construction)
    public void setFlywheel(Flywheel flywheel)
    {
        _flywheel = flywheel;
    }
    
    //--- Set intake reference (call after construction)
    public void setIntake(Intake intake)
    {
        _intake = intake;
    }
    
    //--- Set camera reference (call after construction)
    public void setCamera(Camera camera)
    {
        _camera = camera;
    }
    
    //--- Set lights reference (call after construction)
    public void setLights(Lights lights)
    {
        _lights = lights;
    }
    //endregion

    //region --- Run (call this in your main loop) ---
    public void run()
    {
        //--- Get current position
        int currentPosition = _motor.getCurrentPosition();
        int targetEncoder = getTargetEncoder();
        int error = Math.abs(targetEncoder - currentPosition);

        //--- Update target position (RUN_TO_POSITION handles the movement)
        _motor.setTargetPosition(targetEncoder);
        
        //--- Adjust power based on whether we're moving or holding
        if (error > POSITION_TOLERANCE)
        {
            //--- Moving to target - use higher power
            _motor.setPower(MOTOR_SPEED);
            _isMoving = true;
        }
        else
        {
            //--- At target - use lower holding power to reduce oscillation
            _motor.setPower(HOLDING_POWER);
            _isMoving = false;
        }
    }
    //endregion

    //region --- Control Methods ---

    //--- Call this in your main loop to handle kickstand controls
    //--- GP2 Left Bumper: Toggle between UP and DOWN
    public void controlKickstand()
    {
        //--- GP2 Left Bumper - toggle position
        if (_gamepad2.left_bumper)
        {
            if (!_leftBumperPressed)
            {
                _leftBumperPressed = true;
                toggle();
            }
        }
        else
        {
            _leftBumperPressed = false;
        }
    }

    //--- Toggle between UP and DOWN positions
    public void toggle()
    {
        //--- Turn off flywheel, intake, and camera scanning when using kickstand
        if (_flywheel != null)
        {
            _flywheel.stop();
        }
        if (_intake != null)
        {
            _intake.stop();
        }
        if (_camera != null)
        {
            _camera.pauseScanning();
        }
        
        if (_targetPosition == Position.DOWN)
        {
            _targetPosition = Position.UP;
            //--- Kickstand going UP (off) - other methods will control lights
        }
        else
        {
            _targetPosition = Position.DOWN;
            //--- Kickstand going DOWN (on) - show yellow/orange/red pattern
            if (_lights != null)
            {
                _lights.setLeft(Lights.Color.YELLOW);
                _lights.setMiddle(Lights.Color.ORANGE);
                _lights.setRight(Lights.Color.RED);
            }
        }
    }

    //--- Move to UP position
    public void moveUp()
    {
        _targetPosition = Position.UP;
    }

    //--- Move to DOWN position
    public void moveDown()
    {
        _targetPosition = Position.DOWN;
    }

    //--- Set target position
    public void setPosition(Position position)
    {
        _targetPosition = position;
    }

    //--- Stop the motor immediately
    public void stop()
    {
        _motor.setPower(0);
        _isMoving = false;
    }

    //endregion

    //region --- Public Methods ---

    //--- Get the current target position
    public Position getTargetPosition()
    {
        return _targetPosition;
    }

    //--- Check if currently moving
    public boolean isMoving()
    {
        return _isMoving;
    }

    //--- Check if at target position
    public boolean isAtTarget()
    {
        int currentPosition = _motor.getCurrentPosition();
        int targetEncoder = getTargetEncoder();
        return Math.abs(targetEncoder - currentPosition) <= POSITION_TOLERANCE;
    }

    //--- Check if kickstand is up
    public boolean isUp()
    {
        return _targetPosition == Position.UP && isAtTarget();
    }

    //--- Check if kickstand is down
    public boolean isDown()
    {
        return _targetPosition == Position.DOWN && isAtTarget();
    }

    //--- Get current encoder position
    public int getCurrentEncoderPosition()
    {
        return _motor.getCurrentPosition();
    }

    //--- Get target encoder position
    private int getTargetEncoder()
    {
        return (_targetPosition == Position.UP) ? POSITION_UP : POSITION_DOWN;
    }

    //endregion

    //region --- Telemetry ---

    public void getTelemetry()
    {
        if (_showInfo)
        {
            _telemetry.addData("Kickstand Target", _targetPosition);
            _telemetry.addData("Kickstand Encoder", _motor.getCurrentPosition());
            _telemetry.addData("Kickstand Moving", _isMoving);
            _telemetry.addData("Kickstand Power", "%.2f", _motor.getPower());
        }
    }

    //endregion
}
