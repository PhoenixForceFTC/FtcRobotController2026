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

@TeleOp(name="TeleOp - Alpha", group="1")
public class TeleOp_Alpha extends LinearOpMode
{
    //------------------------------------------------------------------------------------------
    // Variables
    //------------------------------------------------------------------------------------------
    RobotHardware _robot = new RobotHardware(this);
    public ElapsedTime _runtime = new ElapsedTime();
    
    //--- Auto-fire sequence state
    private boolean _gp2RightTriggerPressed = false;
    
    //--- RPM for auto-fire (uses LONG distance lookup)
    private static final double AUTO_FIRE_RPM = 3800.0;  // RPM for 110" distance

    //------------------------------------------------------------------------------------------
    //--- OpMode
    //------------------------------------------------------------------------------------------
    @Override
    public void runOpMode()
    {
        //------------------------------------------------------------------------------------------
        //--- Robot Initialize
        //------------------------------------------------------------------------------------------
        _robot.init(RobotHardware.ALPHA);

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
            //------------------------------------------------------------------------------------------
            double suggestedRPM = _robot.camera.getSuggestedVelocity(_robot.intake.getBallCount());
            if (suggestedRPM > 0)
            {
                _robot.flywheel.setVelocity(suggestedRPM);
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
            //--- Gamepad 2 Right Trigger - Auto-Fire Sequence (RoadRunner controlled)
            //------------------------------------------------------------------------------------------
            if (gamepad2.right_trigger > 0.5)
            {
                if (!_gp2RightTriggerPressed)
                {
                    _gp2RightTriggerPressed = true;
                    runAutoFireSequence();
                }
            }
            else
            {
                _gp2RightTriggerPressed = false;
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
    //--- Triggered by gamepad2 right trigger
    //--- Sets position to (0,0) heading 180°, moves forward 3", rotates to 235°, fires 3 balls
    //------------------------------------------------------------------------------------------
    private void runAutoFireSequence()
    {
        //--- Lock distance to LONG for this shot
        _robot.camera.setFixedDistanceLong();
        
        //--- Show yellow lights during auto-fire sequence
        _robot.lights.setAllYellow();
        
        //--- Set starting pose: (0, 0) heading 180°
        Pose2d startPose = pose(0, 0, 180);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        
        //--- Run the sequence: move forward 3", rotate to 235°, fire all 3 balls
        Actions.runBlocking(
            drive.actionBuilder(startPose)
                //--- Start flywheel
                .stopAndAdd(new AutoActions.FlywheelSetSpeed(_robot, AUTO_FIRE_RPM))
                //--- Move forward 3" and rotate to 235°
                .strafeToSplineHeading(pos(0, 3), degreeHeading(235))
                //--- Wait for flywheel and fire all 3 balls
                .stopAndAdd(new AutoActions.KickerWaitForSpeedThenFireAll(_robot, AUTO_FIRE_RPM, AUTO_FIRE_RPM, AUTO_FIRE_RPM))
                .build()
        );
        
        //--- Unlock distance after firing
        _robot.camera.unlockDistance();
        
        //--- Turn off lights (camera will take over on next run)
        _robot.lights.setAllOff();
    }

}
