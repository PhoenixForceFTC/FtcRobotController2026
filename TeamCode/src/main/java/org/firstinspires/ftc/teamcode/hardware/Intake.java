package org.firstinspires.ftc.teamcode.hardware;

//region --- Imports ---
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
//endregion

public class Intake
{
    //region --- Constants ---
    //--- Intake/Outtake Speed Settings
    private static final double INTAKE_SPEED = 0.75;
    private static final double OUTTAKE_SPEED = 0.33;

    //--- Outtake Duration (in seconds)
    private static final double OUTTAKE_DURATION = 1.0;
    //endregion

    //region --- Hardware ---
    private final DcMotor _motorIntake;
    private final Gamepad _gamepad1;
    private final Gamepad _gamepad2;
    private final Telemetry _telemetry;
    private final boolean _showInfo;
    private final int _robotVersion;
    //endregion

    //region --- State ---
    private boolean _intakeOn = false;
    private boolean _triggerWasPressed = false;
    private boolean _outtakeActive = false;
    private ElapsedTime _outtakeTimer = new ElapsedTime();
    //endregion

    //region --- Constructor ---
    public Intake(
            DcMotor motorIntake,
            Gamepad gamepad1,
            Gamepad gamepad2,
            Telemetry telemetry,
            int robotVersion,
            boolean showInfo
    )
    {
        this._motorIntake = motorIntake;
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
        _intakeOn = false;
        _outtakeActive = false;
        stop();
    }
    //endregion

    //region --- Control Methods ---

    //--- Call this in your main loop to handle intake controls
    //--- Left Trigger: Toggle intake on/off
    //--- Left Bumper: Outtake for 2 seconds then stop
    public void controlIntake()
    {
        //--- Handle outtake timer
        if (_outtakeActive)
        {
            if (_outtakeTimer.seconds() >= OUTTAKE_DURATION)
            {
                //--- Outtake duration complete - stop
                _outtakeActive = false;
                _intakeOn = false;
                stop();
            }
            //--- Still in outtake mode, don't process other inputs
        }
        else
        {
            //--- Handle left bumper - start outtake
            if (_gamepad1.left_bumper)
            {
                _outtakeActive = true;
                _intakeOn = false;
                _outtakeTimer.reset();
                outtake();
            }
            //--- Handle left trigger toggle (detect press, not hold)
            else if (_gamepad1.left_trigger > 0.1)
            {
                if (!_triggerWasPressed)
                {
                    //--- Trigger just pressed - toggle intake
                    _triggerWasPressed = true;
                    _intakeOn = !_intakeOn;

                    if (_intakeOn)
                    {
                        intake();
                    }
                    else
                    {
                        stop();
                    }
                }
            }
            else
            {
                //--- Trigger released
                _triggerWasPressed = false;
            }
        }
    }

    //endregion

    //region --- Public Methods ---

    //--- Run intake at configured speed
    public void intake()
    {
        _motorIntake.setPower(INTAKE_SPEED);
    }

    //--- Run intake at custom speed
    public void intake(double power)
    {
        _motorIntake.setPower(Math.abs(power));
    }

    //--- Run outtake (reverse) at configured speed
    public void outtake()
    {
        _motorIntake.setPower(-OUTTAKE_SPEED);
    }

    //--- Run outtake (reverse) at custom speed
    public void outtake(double power)
    {
        _motorIntake.setPower(-Math.abs(power));
    }

    //--- Stop the intake motor
    public void stop()
    {
        _motorIntake.setPower(0);
    }

    //--- Check if intake is currently running
    public boolean isIntakeOn()
    {
        return _intakeOn;
    }

    //--- Check if outtake is currently active
    public boolean isOuttakeActive()
    {
        return _outtakeActive;
    }

    //endregion

    //region --- Telemetry ---

    public void getTelemetry()
    {
        if (_showInfo)
        {
            _telemetry.addData("Intake Power", "%4.2f", _motorIntake.getPower());
            _telemetry.addData("Intake On", _intakeOn);
            _telemetry.addData("Outtake Active", _outtakeActive);
        }
    }

    //endregion
}
