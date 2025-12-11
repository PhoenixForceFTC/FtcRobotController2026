package org.firstinspires.ftc.teamcode;

//region -- Imports ---

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autos.AutoActions;
import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Lights;

import static org.firstinspires.ftc.teamcode.utils.AutoUtils.pos;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.degreeHeading;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.pose;
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
//  - Right Trigger     - AUTO-FIRE SEQUENCE (RoadRunner controlled)
//                        • Sets position to (0,0) heading 180°
//                        • Moves forward 3", rotates to 235°
//                        • Fires 3 balls at LONG distance
//  - Right Bumper      - (available)
//  - Left Trigger      - (available)
//  - Left Bumper       - Toggle Kickstand Up/Down
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
//  - A (✕)            - Release lock (disable auto-align)
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
//  - A (✕)            - Toggle LONG distance lock (3 orange lights)
//  - X (■)             - (available)
//

//--- TODO manual fire for each of the kickers - left, center, right
//--- TODO check the sequential firing in auto-aim
//--- TODO - easier way to show intake is on (tape?)
//--- TODO look at RPM for far shot

//======================================================================
//endregion

@TeleOp(name="TeleOp - Beta", group="1")
public class TeleOp_Beta extends LinearOpMode
{
    //------------------------------------------------------------------------------------------
    // Variables
    //------------------------------------------------------------------------------------------
    RobotHardware _robot = new RobotHardware(this);
    public ElapsedTime _runtime = new ElapsedTime();
    
    //--- Auto-fire sequence state
    private boolean _gp2RightTriggerPressed = false;
    private boolean _gp2LeftTriggerPressed = false;
    
    //--- RPM for auto-fire (uses LONG distance lookup)
    private static final double AUTO_FIRE_RPM = 3800.0;  // RPM for 110" distance
    
    //--- Turn angles for auto-fire sequences
    private static final double TURN_RIGHT_ANGLE = 154.0;  // Degrees to turn right
    private static final double TURN_LEFT_ANGLE = 206.0;   // Degrees to turn left

    //------------------------------------------------------------------------------------------
    //--- OpMode
    //------------------------------------------------------------------------------------------
    @Override
    public void runOpMode()
    {
        //------------------------------------------------------------------------------------------
        //--- Robot Initialize
        //------------------------------------------------------------------------------------------
        _robot.init(RobotHardware.BETA);

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
        
        //--- Start flywheel at idle speed so first shot is accurate
        _robot.flywheel.setVelocity(2600);

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
            telemetry.addData("Cam", "%s @ %.0f/%.0f RPM", 
                    _robot.camera.getStoredDistanceFormatted(),
                    _robot.flywheel.getCurrentRPM(),
                    _robot.flywheel.getTargetRPM());
            telemetry.addData("Pattern", "%s (%s)", 
                    _robot.kickers.getSequence(),
                    _robot.camera.isSequenceDetected() ? "Detected" : "Default");

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
            //_robot.kickers.getTelemetry();
            //_robot.kickers.fineTunePositions();

            //------------------------------------------------------------------------------------------
            //--- Flywheel - Auto-adjust to suggested velocity based on distance and ball count
            //--- getSuggestedVelocity returns: >0 for valid RPM, 0 when paused, -1 if no distance
            //------------------------------------------------------------------------------------------
            double suggestedRPM = _robot.camera.getSuggestedVelocity(_robot.intake.getBallCount());
            if (suggestedRPM >= 0)
            {
                _robot.flywheel.setVelocity(suggestedRPM);  // 0 = stop when paused
            }
            //_robot.flywheel.testVelocities();

            //------------------------------------------------------------------------------------------
            //--- Camera / Targeting Controls
            //------------------------------------------------------------------------------------------
            _robot.camera.handleTargetingControls();  //--- gp2: Dpad Up/Down=mode, Y/B/A=actions
            //_robot.camera.fineTuneCameraPos();

            //------------------------------------------------------------------------------------------
            //--- Kickstand
            //------------------------------------------------------------------------------------------
            _robot.kickstand.controlKickstand();  //--- gp2: Left Bumper=toggle up/down

            //------------------------------------------------------------------------------------------
            //--- Gamepad 2 Triggers - Auto-Fire Sequences (RoadRunner controlled)
            //------------------------------------------------------------------------------------------
            //--- Right Trigger: Turn RIGHT and fire
            if (gamepad2.right_trigger > 0.5)
            {
                if (!_gp2RightTriggerPressed)
                {
                    _gp2RightTriggerPressed = true;
                    runAutoFireSequence(TURN_RIGHT_ANGLE);
                }
            }
            else
            {
                _gp2RightTriggerPressed = false;
            }
            
            //--- Left Trigger: Turn LEFT and fire
            if (gamepad2.left_trigger > 0.5)
            {
                if (!_gp2LeftTriggerPressed)
                {
                    _gp2LeftTriggerPressed = true;
                    runAutoFireSequence(TURN_LEFT_ANGLE);
                }
            }
            else
            {
                _gp2LeftTriggerPressed = false;
            }

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

    //------------------------------------------------------------------------------------------
    //--- Auto-Fire Sequence (RoadRunner controlled)
    //--- Triggered by gamepad2 triggers
    //--- Sets position to (0,0) heading 180°, rotates to target angle, fires all balls
    //--- @param turnAngle - the heading to rotate to (e.g., 235° for right, 125° for left)
    //------------------------------------------------------------------------------------------
    private void runAutoFireSequence(double turnAngle)
    {
        //--- Lock distance to LONG for this shot
        _robot.camera.setFixedDistanceLong();
        
        //--- Show yellow lights during auto-fire sequence (use MANUAL mode to bypass slot system)
        _robot.lights.forceMode(Lights.LightMode.MANUAL);
        _robot.lights.setAllYellow();
        
        //--- Set starting pose: (0, 0) heading 180°
        Pose2d startPose = pose(0, 0, 180);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        
        //--- Run the sequence: spin up flywheel, rotate to target angle, fire all balls
        Actions.runBlocking(
            drive.actionBuilder(startPose)
                //--- Start flywheel
                .stopAndAdd(new AutoActions.FlywheelSetSpeed(_robot, AUTO_FIRE_RPM))
                //--- Wait 2 seconds for flywheel to spin up
                .waitSeconds(2.0)
                //--- Rotate to target angle
                .turnTo(degreeHeading(turnAngle))
                //--- Wait for flywheel and fire all 3 balls
                .stopAndAdd(new AutoActions.KickerWaitForSpeedThenFireAll(_robot, AUTO_FIRE_RPM, AUTO_FIRE_RPM, AUTO_FIRE_RPM))
                .build()
        );
        
        //--- Unlock distance after firing
        _robot.camera.unlockDistance();
        
        //--- Return to normal mode system (camera will take over)
        _robot.lights.forceMode(Lights.LightMode.CAMERA_TARGET);
    }

}

