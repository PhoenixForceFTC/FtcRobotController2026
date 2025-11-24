package org.firstinspires.ftc.teamcode.hardware;

//region --- Imports ---
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.utils.ServoUtils;
//endregion

public class Intake
{
    public void initialize()
    {
        if (_robotVersion == 1) //--- Alpha
        {

        }
        else //--- Beta
        {
        }
    }

    //region --- Variables ---


    //--- Hardware ---
    private final DcMotor _motorIntake;
    private final Gamepad _gamepad1;
    private final Gamepad _gamepad2;
    private final Telemetry _telemetry;
    private final boolean _showInfo;

    private int _robotVersion;
    //endregion

    //region --- Enums ---

    //endregion

    //region --- Constructor ---
    public Intake(
            DcMotor motorIntake,
            Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, int robotVersion, boolean showInfo
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

    //region --- Public Methods ---

    public void intakeByPowerOut(double power) { powerOut(power); }
    public void intakeByPowerIn(double power) { powerIn(power); }

    //endregion

    //region --- Public Control Mapping ---

    //--- Handles intake motor power based on gamepad input
    public void intakeByPower()
    {
        //--- Configure motors
        MotorUtils.configureForPower(_motorIntake);

        //--- Handle extension and retraction with lift state checks
        if (_gamepad1.left_trigger > 0.1)
        {

            _motorIntake.setPower(_gamepad1.left_trigger); //--- Scale power by trigger pressure

        }
        else if (_gamepad1.left_bumper)
        {
            _motorIntake.setPower(-1); //--- Full power in reverse
        }
        else
        {
            _motorIntake.setPower(0); //--- Stop motorIntake when no input
        }

        //--- Show telemetry if enabled
        if (_showInfo)
        {
            _telemetry.addData("Intake -> Motor Power", "%4.2f", _motorIntake.getPower());
        }
    }

    public void controlIntake() {

        if (_gamepad2.dpad_right)
        {
            intakeByPowerOut(1.0);
        }
        else if (_gamepad2.dpad_left)
        {
            intakeByPowerIn(1.0);
        }
        else
        {
            stop();
        }
    }

    //endregion

    //region --- Private Manual Control Methods ---

    //--- Keeps track of the last control mode

    private void powerOut(double power)
    {
        //--- Ensure power is positive for upward movement
        power = Math.abs(power);

        //--- Call the centralized liftByPower method
        intakeByPower(power);
    }

    //--- Moves the lift down by power and resets the encoder
    private void powerIn(double power)
    {
        //--- Ensure power is positive, then make it negative for downward movement
        power = -Math.abs(power);

        //--- Call the centralized liftByPower method
        intakeByPower(power);
    }

    private void stop()
    {
        MotorUtils.stopMotor(_motorIntake);
    }


    //--- Moves the lift by specified power
    private void intakeByPower(double power)
    {
        //--- Configure motor for no encoder mode
        MotorUtils.configureForPower(_motorIntake);

        //--- Set the power to motor
        MotorUtils.setPower(_motorIntake, power);

        //--- Show telemetry if enabled
        if (_showInfo)
        {
            _telemetry.addData("Intake -> Power", "%4.2f", _motorIntake.getPower());
        }
    }

    //endregion

    //region --- Utility Methods ---

    //--- TODO: Move to util class
    public static void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    //endregion
}
