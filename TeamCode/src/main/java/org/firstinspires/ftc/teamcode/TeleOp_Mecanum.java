package org.firstinspires.ftc.teamcode;

//region -- Imports ---

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Lights;
//endregion

//region --- Controls ---
//======================================================================
// GAMEPAD 1 (Driver) 
//======================================================================
//
//  MOVEMENT:
//  - Left Stick        - Mecanum Drive (strafe/forward/back)
//  - Right Stick       - Mecanum Rotate
//  - Left Stick Click  - Drive Speed High/Low (Hold 1 second)
//  - Right Stick Click - Rotate Speed High/Low (Hold 1 second)
//  - Dpad              - Slow directional movement (Up/Down/Left/Right)
//
//  SHOOTING:
//  - Right Trigger     - Fire All Kickers
//                        • Reads ball count from intake
//                        • Gets RPM from distance lookup table (1/2/3 ball tables)
//                        • Waits for flywheel velocity, then fires
//  - Right Bumper      - Fire Kickers in Sequence
//                        • Uses 1-ball RPM (fires one at a time)
//                        • Reads ball colors from intake
//                        • Fires in order based on detected sequence
//
//  INTAKE:
//  - Left Trigger      - Intake Toggle On/Off
//  - Left Bumper       - Outtake (timed reverse)
//
//  VELOCITY / DISTANCE CONTROLS:
//  - Y (▲)             - Increase RPM adjustment (+50, added to lookup value)
//  - A (✕)             - Decrease RPM adjustment (-50, added to lookup value)
//  - X (■)             - Toggle SHORT distance lock
//                        • Press to lock (LEFT light orange), press again to unlock
//  - B (○)             - Toggle MEDIUM distance lock
//                        • Press to lock (LEFT+CENTER lights orange), press again to unlock
//
//======================================================================
// GAMEPAD 2 (Operator) 
//======================================================================
//
//  MODE SELECTION:
//  - Dpad Up           - Switch to AUTO-AIM mode
//  - Dpad Down         - Switch to MANUAL TARGET mode
//  - Dpad Left         - (available)
//  - Dpad Right        - (available)
//
//  TRIGGERS/BUMPERS:
//  - Right Trigger     - (available)
//  - Right Bumper      - (available)
//  - Left Trigger      - (available)
//  - Left Bumper       - (available)
//
//  STICKS:
//  - Left Stick        - (available)
//  - Right Stick       - (available)
//  - Left Stick Click  - (available)
//  - Right Stick Click - (available)
//
//----------------------------------------------------------------------
//  AUTO-AIM MODE (default):
//----------------------------------------------------------------------
//  Camera tracks target, auto-calculates velocity from distance.
//  Robot auto-rotates to align with target when firing.
//
//  - Y (▲)             - Lock on to target (enable auto-align)
//  - A (✕)             - Release lock (disable auto-align)
//  - B (○)             - (available)
//  - X (■)             - (available)
//
//----------------------------------------------------------------------
//  MANUAL TARGET MODE:
//----------------------------------------------------------------------
//  For shooting without camera tracking.
//  Use distance presets for known shooting positions.
//  Light pattern shows lock level: 1/2/3 orange lights.
//
//  - Y (▲)             - Toggle SHORT distance lock (1 orange light)
//  - B (○)             - Toggle MEDIUM distance lock (2 orange lights)
//  - A (✕)             - Toggle LONG distance lock (3 orange lights)
//  - X (■)             - (available)
//
//======================================================================
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
            //_robot.intake.testColorSensors();  //--- Show color sensor values for tuning

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

