package org.firstinspires.ftc.teamcode;

//region -- Imports ---

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
//endregion

//region --- Controls ---
//----------------------------------------------------------------------
// Joystick 1 -----------------------------------------------------------
//  - Left Stick        - Mecanum Drive
//  - Right Stick       - Mecanum Rotate
//  - Left Stick Click  - Drive Speed High/Low
//  - Right Stick Click - Rotate Speed High/Low
//
//  - Dpad Up           - Move Forward (Slow)
//  - Dpad Down         - Move Back (Slow)
//  - Dpad Right        - Move Right (Slow)
//  - Dpad Left         - Move Left (Slow)
//
//  - Right Trigger     - Close Claw
//  - Right Bumpers     - Open Claw
//  - Left Trigger      - Intake Arm Out
//  - Left Bumpers      - Intake Arm Back
//
//  - Y (▲)             - Next Step in Current Mode
//  - A (✕)             - Previous Step in Current Mode
//  - X (■)             - Intake In/Out
//  - B (○)             -
//
//----------------------------------------------------------------------
// Joystick 2 -----------------------------------------------------------
//  - Left Stick        -
//  - Right Stick       -
//  - Left Stick Click  - Reset Intake Encoder
//  - Right Stick Click - Reset Lift Encoder
//
//  - Dpad Up           - Manual Lift Up
//  - Dpad Down         - Manual Lift Down
//  - Dpad Right        - Manual Intake In
//  - Dpad Left         - Manual Intake Out
//
//  - Right Trigger     -
//  - Right Bumpers     - Loosen Intake Claw
//  - Left Trigger      -
//  - Left Bumpers      - Close Arm Claw

//  - Y (▲)             - Mode -> High Basket
//  - A (✕)             - Mode -> Low Basket
//  - X (■)             - Mode -> Climbing
//  - B (○)             - Mode -> Specimens
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
//            _robot.intake.intakeByEncoder();
//            _robot.intake.intakeByPower();
//            _robot.intake.setLiftArmControls();
//            _robot.intake.controlIntake();
//            _robot.intake.setClawControls();

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