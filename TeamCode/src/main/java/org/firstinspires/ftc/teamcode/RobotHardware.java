package org.firstinspires.ftc.teamcode;

//region --- Imports ---
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Kickers;
import org.firstinspires.ftc.teamcode.hardware.Kickstand;
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
2 - cl (Color Sensor Left)
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
0 - pinpoint (Pinpoint Odometry)
1 - cc (Color Sensor Center)
2 - cr (Color Sensor Right)
3 -
*/
//endregion

public class RobotHardware
{
    //------------------------------------------------------------------------------------------
    //--- Robot Versions
    //------------------------------------------------------------------------------------------
    public static final int ALPHA = 1;
    public static final int BETA = 2;

    //------------------------------------------------------------------------------------------
    //--- Settings
    //------------------------------------------------------------------------------------------
    private static final boolean SHOW_INFO = true;
    private int _robotVersion = BETA;  // Default to Beta

    //------------------------------------------------------------------------------------------
    //--- OpMode
    //------------------------------------------------------------------------------------------
    private final LinearOpMode _opMode;

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
    //--- Sensors
    //------------------------------------------------------------------------------------------
    public HuskyLens huskyLens = null;
    public ColorSensor colorSensorLeft = null;
    public ColorSensor colorSensorCenter = null;
    public ColorSensor colorSensorRight = null;

    //------------------------------------------------------------------------------------------
    //--- Custom Hardware Classes
    //------------------------------------------------------------------------------------------
    public Intake intake;
    public Drive drive;
    public Kickers kickers;
    public Kickstand kickstand;
    public Lights lights;
    public Flywheel flywheel;
    public Camera camera;

    //------------------------------------------------------------------------------------------
    //--- Define a constructor that allows the OpMode to pass a reference to itself
    //------------------------------------------------------------------------------------------
    public RobotHardware(LinearOpMode opmode)
    {
        _opMode = opmode;
    }

    //------------------------------------------------------------------------------------------
    // Initialize all the robot's hardware.
    // This method must be called ONCE when the OpMode is initialized.
    //------------------------------------------------------------------------------------------
    public void init(int robotVersion)
    {
        _robotVersion = robotVersion;

        //------------------------------------------------------------------------------------------
        //--- Motor Config
        //------------------------------------------------------------------------------------------
        //--- Drive Motors
        motorDriveFrontLeft = _opMode.hardwareMap.get(DcMotor.class, "fl");
        motorDriveRearLeft = _opMode.hardwareMap.get(DcMotor.class, "rl");
        motorDriveFrontRight = _opMode.hardwareMap.get(DcMotor.class, "fr");
        motorDriveRearRight = _opMode.hardwareMap.get(DcMotor.class, "rr");

        motorDriveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorDriveRearLeft.setDirection(DcMotor.Direction.REVERSE);
        motorDriveFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorDriveRearRight.setDirection(DcMotor.Direction.FORWARD);

        //--- Utility Motors
        motorIntake = _opMode.hardwareMap.get(DcMotor.class, "in");
        motorKickstand = _opMode.hardwareMap.get(DcMotor.class, "kick");
        motorFlyLeft = _opMode.hardwareMap.get(DcMotor.class, "flyl");
        motorFlyRight = _opMode.hardwareMap.get(DcMotor.class, "flyr");

        motorIntake.setDirection(DcMotor.Direction.FORWARD);
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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

        //--- Flywheel PIDF Tuning
        //--- Default goBILDA 6000 RPM values are P=10, I=3, D=0, F=0
        //--- P=10 (responsive), I=1 (moderate), D=2 (light damping), F=0 (no feedforward)
        PIDFCoefficients flywheelPIDF = new PIDFCoefficients(10, 1, 2, 0);
        ((DcMotorEx) motorFlyLeft).setVelocityPIDFCoefficients(flywheelPIDF.p, flywheelPIDF.i, flywheelPIDF.d, flywheelPIDF.f);
        ((DcMotorEx) motorFlyRight).setVelocityPIDFCoefficients(flywheelPIDF.p, flywheelPIDF.i, flywheelPIDF.d, flywheelPIDF.f);

        //------------------------------------------------------------------------------------------
        //--- Servo Config
        //------------------------------------------------------------------------------------------
        servoKickerLeft = _opMode.hardwareMap.get(Servo.class, "kl");
        servoKickerMiddle = _opMode.hardwareMap.get(Servo.class, "km");
        servoKickerRight = _opMode.hardwareMap.get(Servo.class, "kr");

        servoKickerLeft.setDirection(Servo.Direction.REVERSE);

        servoCameraYaw = _opMode.hardwareMap.get(Servo.class, "yaw");
        servoCameraPitch = _opMode.hardwareMap.get(Servo.class, "pitch");

        servoLightLeft = _opMode.hardwareMap.get(Servo.class, "ltl");
        servoLightMiddle = _opMode.hardwareMap.get(Servo.class, "ltm");
        servoLightRight = _opMode.hardwareMap.get(Servo.class, "ltr");

        //--- HuskyLens
        huskyLens = _opMode.hardwareMap.get(HuskyLens.class, "cam");

        //--- Color Sensors
        colorSensorLeft = _opMode.hardwareMap.get(ColorSensor.class, "cl");
        colorSensorCenter = _opMode.hardwareMap.get(ColorSensor.class, "cc");
        colorSensorRight = _opMode.hardwareMap.get(ColorSensor.class, "cr");

        //------------------------------------------------------------------------------------------
        //--- Hardware Constructors
        //------------------------------------------------------------------------------------------
        intake = new Intake(
                motorIntake,
                _opMode.gamepad1,
                _opMode.gamepad2,
                _opMode.telemetry,
                robotVersion,
                SHOW_INFO
        );
        intake.initialize();

        drive = new Drive(
                motorDriveFrontLeft,
                motorDriveFrontRight,
                motorDriveRearLeft,
                motorDriveRearRight,
                _opMode.gamepad1,
                _opMode.gamepad2,
                _opMode.telemetry,
                robotVersion,
                SHOW_INFO
        );

        lights = new Lights(
                servoLightLeft,
                servoLightMiddle,
                servoLightRight,
                _opMode.gamepad1,
                _opMode.gamepad2,
                _opMode.telemetry,
                robotVersion,
                SHOW_INFO
        );
        lights.initialize();

        //--- Connect color sensors and lights to intake for ball detection
        intake.setColorSensors(colorSensorLeft, colorSensorCenter, colorSensorRight);
        intake.setLights(lights);

        flywheel = new Flywheel(
                motorFlyLeft,
                motorFlyRight,
                _opMode.gamepad1,
                _opMode.gamepad2,
                _opMode.telemetry,
                robotVersion,
                SHOW_INFO
        );
        flywheel.initialize();

        kickstand = new Kickstand(
                motorKickstand,
                _opMode.gamepad1,
                _opMode.gamepad2,
                _opMode.telemetry,
                robotVersion,
                SHOW_INFO
        );
        kickstand.initialize();
        kickstand.setFlywheel(flywheel);
        kickstand.setIntake(intake);
        kickstand.setLights(lights);
        //--- Camera reference set after camera is created (below)
        
        //--- Set kickstand reference on intake (so intake knows not to update lights when kickstand is down)
        intake.setKickstand(kickstand);

        kickers = new Kickers(
                servoKickerLeft,
                servoKickerMiddle,
                servoKickerRight,
                flywheel,
                _opMode.gamepad1,
                _opMode.gamepad2,
                _opMode.telemetry,
                robotVersion,
                SHOW_INFO
        );
        kickers.initialize();

        camera = new Camera(
                huskyLens,
                servoCameraYaw,
                servoCameraPitch,
                kickers,
                lights,
                drive,
                _opMode.gamepad1,
                _opMode.gamepad2,
                _opMode.telemetry,
                robotVersion,
                SHOW_INFO
        );
        camera.initialize();

        //--- Set camera reference on kickers for alignment-based firing and distance-based velocity
        kickers.setCamera(camera);
        
        //--- Set intake reference on kickers for reading ball count and colors
        kickers.setIntake(intake);
        
        //--- Set camera reference on kickstand for pausing scanning
        kickstand.setCamera(camera);

        //------------------------------------------------------------------------------------------
        //--- Robot-Specific Configuration
        //------------------------------------------------------------------------------------------
        applyRobotSpecificConfig();

        //------------------------------------------------------------------------------------------
        //--- Messages
        //------------------------------------------------------------------------------------------
        String robotName = (_robotVersion == ALPHA) ? "ALPHA" : "BETA";
        _opMode.telemetry.addData(">", "Hardware Initialized (" + robotName + ")");
        _opMode.telemetry.update();
    }

    //------------------------------------------------------------------------------------------
    // Apply robot-specific hardware configurations.
    // Add any differences between Alpha and Beta robots here.
    //------------------------------------------------------------------------------------------
    private void applyRobotSpecificConfig()
    {
        if (_robotVersion == ALPHA)
        {
            //--------------------------------------------------------------------------------------
            //--- ALPHA Robot Configuration
            //--------------------------------------------------------------------------------------
            // Example: Flip servo directions if needed
            // servoKickerLeft.setDirection(Servo.Direction.FORWARD);
            
            // Example: Adjust motor directions if needed
            // motorIntake.setDirection(DcMotor.Direction.REVERSE);
        }
        else if (_robotVersion == BETA)
        {
            //--------------------------------------------------------------------------------------
            //--- BETA Robot Configuration (Current Default)
            //--------------------------------------------------------------------------------------
            // All current hardware settings are configured for Beta
            // Add any Beta-specific overrides here if needed
        }
    }

    //------------------------------------------------------------------------------------------
    // Get the current robot version
    //------------------------------------------------------------------------------------------
    public int getRobotVersion()
    {
        return _robotVersion;
    }

    public String getRobotName()
    {
        return (_robotVersion == ALPHA) ? "ALPHA" : "BETA";
    }

    //------------------------------------------------------------------------------------------
    // Run all hardware that requires periodic updates.
    // This method must be called in the main loop of the OpMode.
    //------------------------------------------------------------------------------------------
    public void run()
    {
        intake.run();
        lights.run();
        kickers.run();
        kickstand.run();
        flywheel.run();
        camera.run();
    }
}
