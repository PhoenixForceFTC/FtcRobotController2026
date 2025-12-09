package org.firstinspires.ftc.teamcode.hardware;

//region --- Imports ---
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;
//endregion

public class Kickstand
{
    //region --- Constants ---
    //--- Motor speed (very slow for 30 RPM motor)
    private static final double MOTOR_SPEED = 0.1;

    //--- Encoder positions (tune these after testing)
    //--- Position 0 = starting position (encoder reset on init)
    //--- Positive or negative depends on motor direction - adjust after testing
    private static final int POSITION_DOWN = 0;       // Starting/down position
    private static final int POSITION_UP = 500;       // Up position (tune this value)

    //--- Position tolerance (how close to target before we consider it "there")
    private static final int POSITION_TOLERANCE = 20;
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
    
    //--- Flywheel reference (to turn off when kickstand is used)
    private Flywheel _flywheel = null;
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
        _motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _motor.setPower(0);
        
        _targetPosition = Position.DOWN;
        _isMoving = false;
    }

    //--- Set flywheel reference (call after construction)
    public void setFlywheel(Flywheel flywheel)
    {
        _flywheel = flywheel;
    }
    //endregion

    //region --- Run (call this in your main loop) ---
    public void run()
    {
        //--- Get current position
        int currentPosition = _motor.getCurrentPosition();
        int targetEncoder = getTargetEncoder();

        //--- Check if we've reached the target position
        int error = targetEncoder - currentPosition;
        
        if (Math.abs(error) <= POSITION_TOLERANCE)
        {
            //--- At target, stop motor
            _motor.setPower(0);
            _isMoving = false;
        }
        else
        {
            //--- Move toward target
            _isMoving = true;
            if (error > 0)
            {
                //--- Need to move positive direction
                _motor.setPower(MOTOR_SPEED);
            }
            else
            {
                //--- Need to move negative direction
                _motor.setPower(-MOTOR_SPEED);
            }
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
        //--- Turn off flywheel when using kickstand
        if (_flywheel != null)
        {
            _flywheel.stop();
        }
        
        if (_targetPosition == Position.DOWN)
        {
            _targetPosition = Position.UP;
        }
        else
        {
            _targetPosition = Position.DOWN;
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
