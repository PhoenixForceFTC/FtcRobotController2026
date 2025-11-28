package org.firstinspires.ftc.teamcode.hardware;

//region --- Imports ---
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
//endregion

public class Flywheel
{
    //region --- Constants ---
    //--- Motor specs: goBILDA 5203 Series 1:1 ratio, 6000 RPM, 28 ticks/rev
    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_RPM = 6000.0;

    //--- Velocity matching tolerance (in RPM)
    private static final double VELOCITY_TOLERANCE = 50.0;

    //--- Dpad increment (RPM per press)
    private static final double DPAD_INCREMENT = 100.0;

    //--- Test velocities (RPM)
    private static final double TEST_VELOCITY_Y = 4000.0;
    private static final double TEST_VELOCITY_B = 3000.0;
    private static final double TEST_VELOCITY_X = 2000.0;
    private static final double TEST_VELOCITY_A = 0.0;
    //endregion

    //region --- Hardware ---
    private final DcMotorEx _motorLeft;
    private final DcMotorEx _motorRight;
    private final Gamepad _gamepad1;
    private final Gamepad _gamepad2;
    private final Telemetry _telemetry;
    private final boolean _showInfo;
    private final int _robotVersion;
    //endregion

    //region --- State ---
    private double _targetRPM = 0.0;
    private boolean _isAtTarget = false;
    private ElapsedTime _rampTimer = new ElapsedTime();
    private double _rampStartTime = 0.0;
    private double _lastRampDuration = 0.0;
    private boolean _isRamping = false;

    //--- Input debouncing
    private boolean _yPressed = false;
    private boolean _bPressed = false;
    private boolean _xPressed = false;
    private boolean _aPressed = false;
    private boolean _dpadUpPressed = false;
    private boolean _dpadDownPressed = false;
    //endregion

    //region --- Constructor ---
    public Flywheel(
            DcMotor motorLeft,
            DcMotor motorRight,
            Gamepad gamepad1,
            Gamepad gamepad2,
            Telemetry telemetry,
            int robotVersion,
            boolean showInfo
    )
    {
        //--- Cast to DcMotorEx for velocity control
        this._motorLeft = (DcMotorEx) motorLeft;
        this._motorRight = (DcMotorEx) motorRight;
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
        stop();
    }
    //endregion

    //region --- Run (call this in your main loop) ---
    public void run()
    {
        //--- Check if we've reached target velocity
        double currentRPM = getCurrentRPM();
        boolean wasAtTarget = _isAtTarget;
        _isAtTarget = Math.abs(currentRPM - _targetRPM) <= VELOCITY_TOLERANCE;

        //--- Track ramp time
        if (_isRamping && _isAtTarget)
        {
            _lastRampDuration = _rampTimer.seconds() - _rampStartTime;
            _isRamping = false;
        }
    }
    //endregion

    //region --- Public Methods - Velocity Control ---

    //--- Set target velocity in RPM (both motors)
    public void setVelocity(double rpm)
    {
        //--- Clamp to valid range
        rpm = Math.max(0, Math.min(MAX_RPM, rpm));

        //--- Start ramp timing if velocity changed
        if (rpm != _targetRPM)
        {
            _targetRPM = rpm;
            _rampStartTime = _rampTimer.seconds();
            _isRamping = true;
            _isAtTarget = false;
        }

        //--- Convert RPM to ticks per second
        double ticksPerSecond = rpmToTicksPerSecond(rpm);

        //--- Set velocity on both motors
        _motorLeft.setVelocity(ticksPerSecond);
        _motorRight.setVelocity(ticksPerSecond);
    }

    //--- Stop the flywheels
    public void stop()
    {
        setVelocity(0);
    }

    //--- Get the current target RPM
    public double getTargetRPM()
    {
        return _targetRPM;
    }

    //--- Get the current actual RPM (average of both motors)
    public double getCurrentRPM()
    {
        double leftTPS = _motorLeft.getVelocity();
        double rightTPS = _motorRight.getVelocity();
        double avgTPS = (leftTPS + rightTPS) / 2.0;
        return ticksPerSecondToRPM(avgTPS);
    }

    //--- Get the current RPM of the left motor
    public double getLeftRPM()
    {
        return ticksPerSecondToRPM(_motorLeft.getVelocity());
    }

    //--- Get the current RPM of the right motor
    public double getRightRPM()
    {
        return ticksPerSecondToRPM(_motorRight.getVelocity());
    }

    //--- Check if flywheels are at target velocity
    public boolean isAtTarget()
    {
        return _isAtTarget;
    }

    //--- Get the last ramp-up duration (time to reach target)
    public double getLastRampDuration()
    {
        return _lastRampDuration;
    }

    //endregion

    //region --- Test Mode ---

    //--- Test flywheel velocities using gamepad2
    //--- Y: 4000 RPM
    //--- B: 3000 RPM
    //--- X: 2000 RPM
    //--- A: Stop (0 RPM)
    //--- Dpad Up: Increase by 100 RPM
    //--- Dpad Down: Decrease by 100 RPM
    public void testVelocities()
    {
        //--- Y button - 4000 RPM
        if (_gamepad2.y)
        {
            if (!_yPressed)
            {
                _yPressed = true;
                setVelocity(TEST_VELOCITY_Y);
            }
        }
        else
        {
            _yPressed = false;
        }

        //--- B button - 3000 RPM
        if (_gamepad2.b)
        {
            if (!_bPressed)
            {
                _bPressed = true;
                setVelocity(TEST_VELOCITY_B);
            }
        }
        else
        {
            _bPressed = false;
        }

        //--- X button - 2000 RPM
        if (_gamepad2.x)
        {
            if (!_xPressed)
            {
                _xPressed = true;
                setVelocity(TEST_VELOCITY_X);
            }
        }
        else
        {
            _xPressed = false;
        }

        //--- A button - Stop
        if (_gamepad2.a)
        {
            if (!_aPressed)
            {
                _aPressed = true;
                setVelocity(TEST_VELOCITY_A);
            }
        }
        else
        {
            _aPressed = false;
        }

        //--- Dpad Up - Increase velocity
        if (_gamepad2.dpad_up)
        {
            if (!_dpadUpPressed)
            {
                _dpadUpPressed = true;
                setVelocity(_targetRPM + DPAD_INCREMENT);
            }
        }
        else
        {
            _dpadUpPressed = false;
        }

        //--- Dpad Down - Decrease velocity
        if (_gamepad2.dpad_down)
        {
            if (!_dpadDownPressed)
            {
                _dpadDownPressed = true;
                setVelocity(_targetRPM - DPAD_INCREMENT);
            }
        }
        else
        {
            _dpadDownPressed = false;
        }

        //--- Display telemetry
        _telemetry.addData("--- FLYWHEEL TEST ---", "");
        _telemetry.addData("Controls", "Y=4000, B=3000, X=2000, A=Stop");
        _telemetry.addData("Adjust", "DpadUp=+100, DpadDown=-100");
        _telemetry.addData("------------------------", "");
        _telemetry.addData("Target RPM", "%.0f", _targetRPM);
        _telemetry.addData("Current RPM", "%.0f (L: %.0f, R: %.0f)", 
                getCurrentRPM(), getLeftRPM(), getRightRPM());
        _telemetry.addData("At Target", _isAtTarget ? "YES ✓" : "NO");
        _telemetry.addData("Ramp Time", "%.2f sec", _isRamping ? 
                (_rampTimer.seconds() - _rampStartTime) : _lastRampDuration);
        _telemetry.addData("------------------------", "");
    }

    //endregion

    //region --- Telemetry ---

    public void getTelemetry()
    {
        if (_showInfo)
        {
            _telemetry.addData("Flywheel Target", "%.0f RPM", _targetRPM);
            _telemetry.addData("Flywheel Current", "%.0f RPM", getCurrentRPM());
            _telemetry.addData("Flywheel At Target", _isAtTarget);
        }
    }

    //endregion

    //region --- Utility Methods ---

    //--- Convert RPM to ticks per second
    private double rpmToTicksPerSecond(double rpm)
    {
        return rpm * TICKS_PER_REV / 60.0;
    }

    //--- Convert ticks per second to RPM
    private double ticksPerSecondToRPM(double ticksPerSecond)
    {
        return ticksPerSecond * 60.0 / TICKS_PER_REV;
    }

    //endregion
}
