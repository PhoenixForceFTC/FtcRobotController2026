package org.firstinspires.ftc.teamcode.hardware;

//region --- Imports ---

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//endregion

//@Config
public class Kickers
{
    //FtcDashboard dashboard = FtcDashboard.getInstance();

    public void initialize()
    {
        if (_robotVersion == 1) //--- Alpha
        {

        }
        else //--- Beta
        {
        }
    }

    //--- Hardware ---
    private final Servo _servoKickerLeft;
    private final Servo _servoKickerMiddle;
    private final Servo _servoKickerRight;
    private final Gamepad _gamepad1;
    private final Gamepad _gamepad2;
    private final Telemetry _telemetry;
    private final boolean _showInfo;

    private int _robotVersion;

    //region --- Enums ---

    //endregion

    //region --- Constructor ---
    public Kickers(
            Servo servoKickerLeft,
            Servo servoKickerMiddle,
            Servo servoKickerRight,
            Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, int robotVersion, boolean showInfo
    )
    {
        this._servoKickerLeft = servoKickerLeft;
        this._servoKickerMiddle = servoKickerMiddle;
        this._servoKickerRight = servoKickerRight;
        this._gamepad1 = gamepad1;
        this._gamepad2 = gamepad2;
        this._telemetry = telemetry;
        this._robotVersion = robotVersion;
        this._showInfo = showInfo;
    }
    //endregion


    private final double positionServoUpLeft = 0.3;
    private final double positionServoDownLeft = 0.5;

    private final double positionServoUpMiddle = 0.7;
    private final double positionServoDownMiddle = 0.5;

    private final double positionServoUpRight = 0.7;
    private final double positionServoDownRight = 0.5;

    // delay before going to zero position
    private final double KICKER_ACTION_DELAY = 1.0;
    private final double GLOBAL_ACTION_DELAY = 0.1;

    private ElapsedTime timerL = new ElapsedTime(2);
    private ElapsedTime timerM = new ElapsedTime(2);
    private ElapsedTime timerR = new ElapsedTime(2);
    private ElapsedTime timerGlobal = new ElapsedTime(2);

    //endregion

    public void run(double targetSpeed, double speed, boolean run){

        if (speed/targetSpeed>0.9 && run)
        {
            if (_gamepad1.dpad_left)
            {
                fireKicker(0);
            }
            else if (_gamepad1.dpad_up)
            {
                fireKicker(1);
            }
            else if (_gamepad1.dpad_right)
            {
                fireKicker(2);
            }
            else if (_gamepad1.dpad_down)
            {
                fireKicker(3);
            }
        }

        if (timerL.seconds() < KICKER_ACTION_DELAY)
        {
            _servoKickerLeft.setPosition(positionServoUpLeft);
        }
        else
        {
            _servoKickerLeft.setPosition(positionServoDownLeft);
        }

        if (timerM.seconds() < KICKER_ACTION_DELAY)
        {
            _servoKickerMiddle.setPosition(positionServoUpMiddle);
        }
        else
        {
            _servoKickerMiddle.setPosition(positionServoDownMiddle);
        }

        if (timerR.seconds() < KICKER_ACTION_DELAY)
        {
            _servoKickerRight.setPosition(positionServoUpRight);
        }
        else
        {
            _servoKickerRight.setPosition(positionServoDownRight);
        }
    }

//    public int[] fireAutoKickerSeq(LimelightHardware2Axis.Motif targetMotif, LimelightHardware2Axis.Motif intakeMotif)
//    {
//        char[] targetMotifSeq = targetMotif.toString().toCharArray();
//        char[] intakeMotifSeq = intakeMotif.toString().toCharArray();
//        int[] seqArr = new int[3];
//        int count = 0;
//        for (char c : targetMotifSeq) {
//            int pos = findChar(intakeMotifSeq, c);
//            if (pos>-1 && pos<3) {
//                seqArr[count++]=pos;
//                intakeMotifSeq[pos] = 'x';
//            }
//        }
//
//        return seqArr;
//    }

    private static int findChar(char[] arr, char target) {
        if (arr == null) {
            return -1; // handle null array
        }

        for (int i = 0; i < arr.length; i++) {
            if (arr[i] == target) {
                return i; // found, return index
            }
        }

        return -1; // not found
    }

    // 0 = leftmost kicker, 1 = middle kicker, 2 = rightmost kicker, 3 = all kickers
    public void fireKicker(int kickerPos)
    {
        if (kickerPos == 0 && timerGlobal.seconds() > GLOBAL_ACTION_DELAY)
        {
            timerL.reset();
            timerGlobal.reset();
        }
        else if (kickerPos==1 && timerGlobal.seconds() > GLOBAL_ACTION_DELAY)
        {
            timerM.reset();
            timerGlobal.reset();
        }
        else if (kickerPos==2 && timerGlobal.seconds() > GLOBAL_ACTION_DELAY)
        {
            timerR.reset();
            timerGlobal.reset();
        }
        else if (kickerPos==3 && timerGlobal.seconds() > GLOBAL_ACTION_DELAY)
        {
            timerL.reset();
            timerM.reset();
            timerR.reset();
            timerGlobal.reset();
        }
    }

    public void fireKickerAuto(int kickerPos)
    {
        if (kickerPos == 0)
        {
            _servoKickerLeft.setPosition(positionServoUpLeft);
        }
        else if (kickerPos == 1)
        {
            _servoKickerMiddle.setPosition(positionServoUpMiddle);
        }
        else if (kickerPos == 2)
        {
            _servoKickerRight.setPosition(positionServoUpRight);
        }
    }

    public void retractKickerAuto(int kickerPos)
    {
        if (kickerPos == 0)
        {
            _servoKickerLeft.setPosition(positionServoDownLeft);
        }
        else if (kickerPos == 1)
        {
            _servoKickerMiddle.setPosition(positionServoDownMiddle);
        }
        else if (kickerPos == 2)
        {
            _servoKickerRight.setPosition(positionServoDownRight);
        }
    }

    public void getTelemetry()
    {
        if(_showInfo)
        {
            _telemetry.addData("Kicker 1 Position: ", _servoKickerLeft.getPosition());
            _telemetry.addData("Kicker 2 Position: ", _servoKickerMiddle.getPosition());
            _telemetry.addData("Kicker 3 Position: ", _servoKickerRight.getPosition());
        }
    }
}