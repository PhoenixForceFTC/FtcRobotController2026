package org.firstinspires.ftc.teamcode;

//region -- Imports ---

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Lights;
//endregion

//region --- Controls ---
//----------------------------------------------------------------------
// Joystick 1 (gp1) -----------------------------------------------------
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
//  - Y (▲)             - Increase Target Velocity (+50 RPM)
//  - A (✕)             - Decrease Target Velocity (-50 RPM)
//  - X (■)             - Set Flywheel Velocity 2000 RPM
//  - B (○)             - Set Flywheel Velocity 3000 RPM
//
//----------------------------------------------------------------------
// Joystick 2 (gp2) -----------------------------------------------------
//  - Left Stick        -
//  - Right Stick       -
//  - Left Stick Click  - 
//  - Right Stick Click - 
//
//  - Dpad Up           - Switch to AUTO-AIM mode
//  - Dpad Down         - Switch to MANUAL TARGET mode
//  - Dpad Right        - 
//  - Dpad Left         - 
//
//  - Right Trigger     -
//  - Right Bumpers     -
//  - Left Trigger      -
//  - Left Bumpers      -
//
//  AUTO-AIM MODE (Dpad Up):
//  - Y (▲)             - Lock on to target (auto-align)
//  - A (✕)             - Release lock (stop auto-align)
//
//  MANUAL TARGET MODE (Dpad Down):
//  - Y (▲)             - Close shot velocity (2700 RPM)
//  - B (○)             - Medium shot velocity (2850 RPM)
//  - A (✕)             - Long shot velocity (3000 RPM)
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
        telemetry.addData("STATUS", "Initialized");
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
            telemetry.addData("STATUS", "Run Time: " + _runtime.toString());

            //------------------------------------------------------------------------------------------
            //--- Drive
            //------------------------------------------------------------------------------------------
            _robot.drive.driveControl(0.5); //--- Both D-pad for directional movement and Joysticks for mecanum movement

            //------------------------------------------------------------------------------------------
            //--- Intake
            //------------------------------------------------------------------------------------------
            _robot.intake.controlIntake();
            _robot.intake.testColorSensors();  //--- Show color sensor values for tuning

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
            //--- Camera / Targeting Controls
            //------------------------------------------------------------------------------------------
            _robot.camera.handleTargetingControls();  //--- gp2: Dpad Up/Down=mode, Y/B/A=actions
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

