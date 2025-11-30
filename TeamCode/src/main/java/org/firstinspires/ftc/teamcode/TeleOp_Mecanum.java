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
//  - Right Trigger     - Fire All Kickers (waits for flywheel velocity)
//  - Right Bumpers     - Fire Kickers in Sequence (waits for velocity between shots)
//  - Left Trigger      - Intake Toggle On/Off
//  - Left Bumpers      - Intake Outtakes for timed duration
//
//  - Y (▲)             - Set Flywheel Velocity 4000 RPM
//  - A (✕)             - Set Flywheel Velocity 1500 RPM
//  - X (■)             - Set Flywheel Velocity 2000 RPM
//  - B (○)             - Set Flywheel Velocity 3000 RPM
//
//----------------------------------------------------------------------
// Joystick 2 -----------------------------------------------------------
//  - Left Stick        -
//  - Right Stick       -
//  - Left Stick Click  - ??Reset Intake Encoder
//  - Right Stick Click - ??Reset Lift Encoder
//
//  - Dpad Up           - Camera Pitch Up / Flywheel +100 RPM
//  - Dpad Down         - Camera Pitch Down / Flywheel -100 RPM
//  - Dpad Right        - Camera Yaw Right
//  - Dpad Left         - Camera Yaw Left
//
//  - Right Trigger     -
//  - Right Bumpers     -
//  - Left Trigger      -
//  - Left Bumpers      -

//  - Y (▲)             - Camera Pitch Up / Flywheel 4000 RPM
//  - A (✕)             - Camera Pitch Down / Flywheel Stop
//  - X (■)             - Camera Yaw Left / Flywheel 2000 RPM
//  - B (○)             - Camera Yaw Right / Flywheel 3000 RPM
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
        _robot.intake.initialize();
        _robot.lights.initialize();

        //------------------------------------------------------------------------------------------
        //--- Camera Mode (switch from PRE_MATCH to TELEOP)
        //------------------------------------------------------------------------------------------
        _robot.camera.setModeTeleOp();  //--- Goals only, enable scanning

        //------------------------------------------------------------------------------------------
        //--- Run until the end of the match (driver presses STOP)
        //------------------------------------------------------------------------------------------
        while (opModeIsActive()) {

            //------------------------------------------------------------------------------------------
            //--- Hardware Run (updates lights, etc.)
            //------------------------------------------------------------------------------------------
            _robot.run();
           
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
            //--- Kickers
            //------------------------------------------------------------------------------------------
            _robot.kickers.controlKickers();
            _robot.kickers.getTelemetry();
            //_robot.kickers.fineTunePositions();

            //------------------------------------------------------------------------------------------
            //--- Flywheel
            //------------------------------------------------------------------------------------------
            //_robot.flywheel.testVelocities();

            //------------------------------------------------------------------------------------------
            //--- Camera
            //------------------------------------------------------------------------------------------
            _robot.camera.handleAlignmentControls();  //--- Y to enable auto-align, A to disable
            //_robot.camera.fineTuneCameraPos();

            //------------------------------------------------------------------------------------------
            //--- Lights
            //------------------------------------------------------------------------------------------
            // _robot.lights.testColors();
            // _robot.lights.testPattern();
 
            //------------------------------------------------------------------------------------------
            //--- Update Telemetry Display
            //------------------------------------------------------------------------------------------
            telemetry.update();
        }
    }

}

