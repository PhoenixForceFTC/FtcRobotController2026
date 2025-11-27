package org.firstinspires.ftc.teamcode;

//region -- Imports ---

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Lights;
//endregion

//region --- Controls ---
//----------------------------------------------------------------------
// Joystick 1 -----------------------------------------------------------
//  - Left Stick        - Mecanum Drive
//  - Right Stick       - Mecanum Rotate
//  - Left Stick Click  - Drive Speed High/Low (Hold 1 second)
//  - Right Stick Click - Rotate Speed High/Low (Hold 1 second)
//
//  - Dpad Up           - Move Forward (Slow)
//  - Dpad Down         - Move Back (Slow)
//  - Dpad Right        - Move Right (Slow)
//  - Dpad Left         - Move Left (Slow)
//
//  - Right Trigger     - Fire 3
//  - Right Bumpers     - Fire 1
//  - Left Trigger      - Intake Toggle On/Off
//  - Left Bumpers      - Intake Outtake for seconds
//
//  - Y (▲)             -
//  - A (✕)             -
//  - X (■)             -
//  - B (○)             - Auto Shoot Long
//
//----------------------------------------------------------------------
// Joystick 2 -----------------------------------------------------------
//  - Left Stick        -
//  - Right Stick       -
//  - Left Stick Click  - ??Reset Intake Encoder
//  - Right Stick Click - ??Reset Lift Encoder
//
//  - Dpad Up           - ??Manual Lift Up
//  - Dpad Down         - ??Manual Lift Down
//  - Dpad Right        - ??Manual Intake In
//  - Dpad Left         - ??Manual Intake Out
//
//  - Right Trigger     -
//  - Right Bumpers     -
//  - Left Trigger      -
//  - Left Bumpers      -

//  - Y (▲)             - ??Mode -> High Basket
//  - A (✕)             - ??Mode -> Low Basket
//  - X (■)             - ??Mode -> Climbing
//  - B (○)             - ??Mode -> Specimens
//----------------------------------------------------------------------
//endregion

@TeleOp(name="TeleOp", group="1")
public class TeleOp_Mecanum extends LinearOpMode
{
    //------------------------------------------------------------------------------------------
    // Variables
    //------------------------------------------------------------------------------------------
    RobotHardware _robot = new RobotHardware(this);
    public ElapsedTime _runtime = new ElapsedTime();

    //------------------------------------------------------------------------------------------
    //--- OpMode
    //------------------------------------------------------------------------------------------
    @Override
    public void runOpMode()
    {
        //------------------------------------------------------------------------------------------
        //--- Robot Initialize
        //------------------------------------------------------------------------------------------
        int robotVersion = 2; //--- 1 for ALPHA and 2 for BETA
        _robot.init(robotVersion);

        //------------------------------------------------------------------------------------------
        //--- Display and wait for the game to start (driver presses START)
        //------------------------------------------------------------------------------------------
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        _runtime.reset();

        //------------------------------------------------------------------------------------------
        //--- Hardware Initialize
        //------------------------------------------------------------------------------------------
//        _robot.arm.initialize();
//        _robot.intake.initialize();
//        _robot.lift.initialize();

        //------------------------------------------------------------------------------------------
        //--- Run until the end of the match (driver presses STOP)
        //------------------------------------------------------------------------------------------
        while (opModeIsActive()) {

            //------------------------------------------------------------------------------------------
            //--- Hardware Run (updates lights, etc.)
            //------------------------------------------------------------------------------------------
            _robot.run();

            //------------------------------------------------------------------------------------------
            //--- Test Lights (uncomment to test light colors with gamepad2 dpad)
            //------------------------------------------------------------------------------------------
            // _robot.lights.testColors();
            
            //------------------------------------------------------------------------------------------
            //--- Start Telemetry Display
            //------------------------------------------------------------------------------------------
            telemetry.addData("Status", "Run Time: " + _runtime.toString());

            //------------------------------------------------------------------------------------------
            //--- Drive
            //------------------------------------------------------------------------------------------
            _robot.drive.driveControl(0.5); //--- Both D-pad for directional movement and Joysticks for mecanum movement

            //------------------------------------------------------------------------------------------
            //--- Intake
            //------------------------------------------------------------------------------------------
            _robot.intake.controlIntake();

            //------------------------------------------------------------------------------------------
            //--- Test Lights Pattern (comment out when done testing)
            //------------------------------------------------------------------------------------------
            // _robot.lights.testPattern();

            //------------------------------------------------------------------------------------------
            //--- Arm
            //------------------------------------------------------------------------------------------
//            _robot.arm.controlArm();
//            _robot.arm.controlArmManual();

            //------------------------------------------------------------------------------------------
            //--- Update Telemetry Display
            //------------------------------------------------------------------------------------------
            telemetry.update();
        }
    }

}

