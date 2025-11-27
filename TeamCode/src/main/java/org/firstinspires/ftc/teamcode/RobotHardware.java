package org.firstinspires.ftc.teamcode;

//region --- Imports ---
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Kickers;
import org.firstinspires.ftc.teamcode.hardware.Lights;
//endregion

//region --- Control Hub Config ---
/*
Motor
0 - kick (Kickstand)
1 - fl (Front Left)
2 - rl (Rear Left)
3 - flyl (Fly Wheel Left)

Servo
0 - yaw (Camera Yaw)
1 - pitch (Camera Pitch)
2 -
3 -
4 - ltl (Light Left)
5 - ltm (Light Middle)

I2C
0 - imu
1 -
2 -
3 - cam (Camera)

*/
//endregion

//region --- Expansion Hub Config ---
/*
Motor
0 - rr (Rear Right)
1 - fr (Front Right)
2 - in (Intake)
3 - flyr (Fly Wheel Right)

Servo
0 - kr (Kicker Right)
1 - kl (Kicker Left)
2 - ltr (Light Right)
3 -
4 -
5 - km (Kicker Middle)

I2C
0 - odo (Pinpoint Odometry)
1 -
2 -
3 -
*/
//endregion

public class RobotHardware {

    //------------------------------------------------------------------------------------------
    //--- Settings
    //------------------------------------------------------------------------------------------
    boolean _showInfo = true;

    //------------------------------------------------------------------------------------------
    //--- OpMode
    //------------------------------------------------------------------------------------------
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    //------------------------------------------------------------------------------------------
    //--- Drive Motors
    //------------------------------------------------------------------------------------------
    public DcMotor motorDriveFrontRight = null;
    public DcMotor motorDriveFrontLeft = null;
    public DcMotor motorDriveRearRight = null;
    public DcMotor motorDriveRearLeft = null;

    //------------------------------------------------------------------------------------------
    //--- Utility Motors
    //------------------------------------------------------------------------------------------
    public DcMotor motorIntake = null;
    public DcMotor motorKickstand = null;
    public DcMotor motorFlyLeft = null;
    public DcMotor motorFlyRight = null;

    //------------------------------------------------------------------------------------------
    //--- Servos
    //------------------------------------------------------------------------------------------
    public Servo servoKickerLeft = null;
    public Servo servoKickerMiddle = null;
    public Servo servoKickerRight = null;

    public Servo servoCameraYaw = null;
    public Servo servoCameraPitch = null;

    public Servo servoLightLeft = null;
    public Servo servoLightMiddle = null;
    public Servo servoLightRight = null;

    //------------------------------------------------------------------------------------------
    //--- Custom Hardware Classes
    //------------------------------------------------------------------------------------------
    public Intake intake;
    public Drive drive;
    public Kickers kickers;
    public Lights lights;

    //------------------------------------------------------------------------------------------
    //--- Define a constructor that allows the OpMode to pass a reference to itself
    //------------------------------------------------------------------------------------------
    public RobotHardware(LinearOpMode opmode)
    {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     */
    public void init(int robotVersion)
    {
        //------------------------------------------------------------------------------------------
        //--- Motor Config
        //------------------------------------------------------------------------------------------
        //--- Drive Motors
        motorDriveFrontLeft = myOpMode.hardwareMap.get(DcMotor.class, "fl");
        motorDriveRearLeft = myOpMode.hardwareMap.get(DcMotor.class, "rl");
        motorDriveFrontRight = myOpMode.hardwareMap.get(DcMotor.class, "fr");
        motorDriveRearRight = myOpMode.hardwareMap.get(DcMotor.class, "rr");

        motorDriveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorDriveRearLeft.setDirection(DcMotor.Direction.REVERSE);
        motorDriveFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorDriveRearRight.setDirection(DcMotor.Direction.FORWARD);

        //--- Utility Motors
        motorIntake = myOpMode.hardwareMap.get(DcMotor.class, "in");
        motorKickstand = myOpMode.hardwareMap.get(DcMotor.class, "kick");
        motorFlyLeft = myOpMode.hardwareMap.get(DcMotor.class, "flyl");
        motorFlyRight = myOpMode.hardwareMap.get(DcMotor.class, "flyr");

        motorIntake.setDirection(DcMotor.Direction.FORWARD);

        motorKickstand.setDirection(DcMotor.Direction.FORWARD);
        motorKickstand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorKickstand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorKickstand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFlyLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFlyLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFlyLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFlyLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motorFlyRight.setDirection(DcMotor.Direction.FORWARD);
        motorFlyRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFlyRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFlyRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //------------------------------------------------------------------------------------------
        //--- Servo Config
        //------------------------------------------------------------------------------------------
        servoKickerLeft = myOpMode.hardwareMap.get(Servo.class, "kl");
        servoKickerMiddle = myOpMode.hardwareMap.get(Servo.class, "km");
        servoKickerRight = myOpMode.hardwareMap.get(Servo.class, "kr");

        servoCameraYaw = myOpMode.hardwareMap.get(Servo.class, "yaw");
        servoCameraPitch = myOpMode.hardwareMap.get(Servo.class, "pitch");

        servoLightLeft = myOpMode.hardwareMap.get(Servo.class, "ltl");
        servoLightMiddle = myOpMode.hardwareMap.get(Servo.class, "ltm");
        servoLightRight = myOpMode.hardwareMap.get(Servo.class, "ltr");

        servoKickerLeft.setDirection(Servo.Direction.REVERSE);

        //------------------------------------------------------------------------------------------
        //--- Hardware Constructors
        //------------------------------------------------------------------------------------------
        intake = new Intake(
                motorIntake,
                myOpMode.gamepad1,
                myOpMode.gamepad2,
                myOpMode.telemetry,
                robotVersion,
                _showInfo
        );
        intake.initialize();

        drive = new Drive(
                motorDriveFrontLeft,
                motorDriveFrontRight,
                motorDriveRearLeft,
                motorDriveRearRight,
                myOpMode.gamepad1,
                myOpMode.gamepad2,
                myOpMode.telemetry,
                robotVersion,
                _showInfo
        );

        kickers = new Kickers(
                servoKickerLeft,
                servoKickerMiddle,
                servoKickerRight,
                myOpMode.gamepad1,
                myOpMode.gamepad2,
                myOpMode.telemetry,
                robotVersion,
                _showInfo
        );
        kickers.initialize();

        lights = new Lights(
                servoLightLeft,
                servoLightMiddle,
                servoLightRight,
                myOpMode.gamepad1,
                myOpMode.gamepad2,
                myOpMode.telemetry,
                robotVersion,
                _showInfo
        );
        lights.initialize();

        //------------------------------------------------------------------------------------------
        //--- Messages
        //------------------------------------------------------------------------------------------
        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Run all hardware that requires periodic updates.
     * This method must be called in the main loop of the OpMode.
     */
    public void run()
    {
        lights.run();
    }
}
