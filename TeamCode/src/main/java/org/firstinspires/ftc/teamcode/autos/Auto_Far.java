package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Lights;

import java.util.ArrayList;
import java.util.List;

import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.utils.AutoUtils.pos;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.degreeHeading;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.pose;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.verySlow;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.slowAccel;

/**
 * Auto_Far
 * 
 * Simplified autonomous routine for FAR starting position:
 * - Press X on gamepad2 during init to select BLUE alliance (default)
 * - Press B on gamepad2 during init to select RED alliance
 * - Press Y on gamepad2 to select PARK (default)
 * - Press A on gamepad2 to select NO PARK
 * - Press Dpad Left/Right to select stack collection (PRELOADS_ONLY or STACK_1)
 * - Press RB to toggle firing mode (SEQUENCE/ALL)
 * - Press Dpad Up/Down to adjust start delay
 * 
 * Routine:
 * 1. Drive from start (1) to shoot position (2)
 * 2. Fire preloads
 * 3. Optionally collect STACK_1 (far stack at X=36)
 * 4. Return to (2) and fire again if stack collected
 * 5. Park at (12, -12)
 */
@Autonomous(name = "Auto: Far", group = "Auto")
public class Auto_Far extends LinearOpMode {

    //--- Robot hardware
    private RobotHardware robot;
    
    //--- Flywheel speeds for shooting (RPM) - varies by ball count
    private static final double SHOOT_1_RPM = 2250.0;  // Single ball
    private static final double SHOOT_2_RPM = 2450.0;  // Two balls
    private static final double SHOOT_3_RPM = 2900.0;  // Three balls

    //--- Log of firing speeds for each phase
    private List<String> fireLog = new ArrayList<>();

    //--- Alliance selection
    private enum Alliance { BLUE, RED }
    private Alliance selectedAlliance = Alliance.BLUE;  // Default to blue

    //--- Parking position selection
    private enum ParkPosition { PARK, NONE }
    private ParkPosition selectedPark = ParkPosition.PARK;  // Default to park

    //--- Stack selection (simplified - only preloads or one stack)
    private enum StackSelection { PRELOADS_ONLY, STACK_1 }
    private StackSelection selectedStacks = StackSelection.PRELOADS_ONLY;
    private boolean dpadLeftPressed = false;
    private boolean dpadRightPressed = false;

    //--- Firing mode selection (sequence = one at a time based on color, all = fire all at once)
    private enum FiringMode { SEQUENCE, ALL }
    private FiringMode selectedFiringMode = FiringMode.ALL;
    private boolean rightBumperPressed = false;

    //--- Programmable delay (seconds) - adjustable with dpad up/down
    private int startDelaySeconds = 0;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    //--- Auto timer for logging when balls are fired
    private ElapsedTime autoTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException 
    {
        //--- Initialize robot hardware
        robot = new RobotHardware(this);
        robot.init(2);  // Beta robot

        //--- Use MANUAL mode for pre-match lights (we control them directly)
        robot.lights.setModeEnabled(false);
        
        //--- Set initial lights based on default alliance
        if (selectedAlliance == Alliance.BLUE)
        {
            robot.lights.setAllBlue();
        }
        else
        {
            robot.lights.setAllRed();
        }

        //========================================================================
        //--- Selection Loop (during init, before start)
        //========================================================================
        while (!isStarted() && !isStopRequested()) 
        {
            //--- Update ball color sensors (force detection even when intake not running)
            robot.intake.updateBallDetection();
            
            //--- Check for alliance selection buttons on gamepad2
            if (gamepad2.x) 
            {
                if (selectedAlliance != Alliance.BLUE)
                {
                    selectedAlliance = Alliance.BLUE;
                    robot.lights.setAllBlue();
                }
            } 
            else if (gamepad2.b) 
            {
                if (selectedAlliance != Alliance.RED)
                {
                    selectedAlliance = Alliance.RED;
                    robot.lights.setAllRed();
                }
            }

            //--- Check for parking position selection on gamepad2
            if (gamepad2.y)
            {
                selectedPark = ParkPosition.PARK;
            }
            else if (gamepad2.a)
            {
                selectedPark = ParkPosition.NONE;
            }

            //--- Toggle firing mode with right bumper (with debounce)
            if (gamepad2.right_bumper)
            {
                if (!rightBumperPressed)
                {
                    rightBumperPressed = true;
                    selectedFiringMode = (selectedFiringMode == FiringMode.SEQUENCE) 
                        ? FiringMode.ALL : FiringMode.SEQUENCE;
                }
            }
            else
            {
                rightBumperPressed = false;
            }

            //--- Adjust start delay with dpad up/down (with debounce)
            if (gamepad2.dpad_up)
            {
                if (!dpadUpPressed)
                {
                    dpadUpPressed = true;
                    if (startDelaySeconds < 120) startDelaySeconds++;
                }
            }
            else
            {
                dpadUpPressed = false;
            }

            if (gamepad2.dpad_down)
            {
                if (!dpadDownPressed)
                {
                    dpadDownPressed = true;
                    if (startDelaySeconds > 0) startDelaySeconds--;
                }
            }
            else
            {
                dpadDownPressed = false;
            }

            //--- Adjust stack selection with dpad left/right (with debounce)
            if (gamepad2.dpad_left)
            {
                if (!dpadLeftPressed)
                {
                    dpadLeftPressed = true;
                    selectedStacks = StackSelection.PRELOADS_ONLY;
                }
            }
            else
            {
                dpadLeftPressed = false;
            }

            if (gamepad2.dpad_right)
            {
                if (!dpadRightPressed)
                {
                    dpadRightPressed = true;
                    selectedStacks = StackSelection.STACK_1;
                }
            }
            else
            {
                dpadRightPressed = false;
            }

            //--- Display current selection and detection status
            telemetry.addData("=== AUTO: FAR ===", "");
            telemetry.addLine("");
            telemetry.addData("=== ALLIANCE SELECTION ===", "");
            telemetry.addData("Press", "X=BLUE, B=RED");
            telemetry.addData(">>> SELECTED", selectedAlliance);
            telemetry.addData("=== PARKING ===", "");
            telemetry.addData("Press", "Y=PARK, A=NONE");
            telemetry.addData(">>> PARK", selectedPark);
            telemetry.addData("=== STACK SELECTION ===", "");
            telemetry.addData("Dpad Left/Right", "PRELOADS / STACK_1");
            telemetry.addData(">>> COLLECT", selectedStacks);
            telemetry.addData("=== FIRING MODE ===", "");
            telemetry.addData("RB", "Toggle SEQUENCE/ALL");
            telemetry.addData(">>> FIRE", selectedFiringMode);
            telemetry.addData("=== START DELAY ===", "");
            telemetry.addData("Dpad Up/Down", "+/- 1 second");
            telemetry.addData(">>> DELAY", "%d seconds", startDelaySeconds);
            telemetry.addData("=== BALL COLORS (L M R) ===", "");
            telemetry.addData(">>> SENSING", robot.kickers.getBallColorsFromSensors());
            telemetry.addLine("");
            telemetry.addData("Status", "Waiting for START...");
            telemetry.update();
        }

        if (isStopRequested()) return;

        //--- Start auto timer for fire log timestamps
        autoTimer.reset();

        //--- Show detected ball colors on lights
        showBallColorsOnLights();

        //========================================================================
        //--- Run selected alliance routine
        //========================================================================
        if (selectedAlliance == Alliance.BLUE) 
        {
            runBlueAlliance();
        } 
        else 
        {
            runRedAlliance();
        }

        //--- Final telemetry with firing log
        telemetry.addData("Status", "Autonomous Complete!");
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addLine("=== FIRING LOG ===");
        for (String entry : fireLog) 
        {
            telemetry.addLine(entry);
        }
        telemetry.update();
        
        //--- Keep running while op mode is active
        while (opModeIsActive()) 
        {
            sleep(100);
        }
    }

    //========================================================================
    //--- BLUE ALLIANCE ROUTINE
    //========================================================================
    private void runBlueAlliance() 
    {
        //--- Starting position (1) - Far side for Blue
        Pose2d startPose = pose(60, -12, 180);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        //========================================================================
        //--- Drive to shoot position (2) and fire preloads
        //========================================================================
        Actions.runBlocking(
            drive.actionBuilder(startPose)
                //--- Start flywheel while driving
                .stopAndAdd(new AutoActions.FlywheelSetSpeed(robot, SHOOT_3_RPM-50))
                //--- Programmable delay
                .waitSeconds(startDelaySeconds)
                //--- Drive to shoot position (2) - pointing at blue goal
                .strafeToSplineHeading(pos(-12, -12), degreeHeading(225))
                //--- Give flywheel time to spin up
                .waitSeconds(1.0)
                //--- Fire preloads
                .stopAndAdd(getFireAction("Preloads"))
                .build()
        );

        //========================================================================
        //--- Stack 1 (far stack at X=36) - if selected
        //========================================================================
        if (selectedStacks == StackSelection.STACK_1)
        {
            Actions.runBlocking(
                drive.actionBuilder(pose(-12, -12, 225))
                    //--- Align with balls (Stack at X=36)
                    .strafeToSplineHeading(pos(36, -16), degreeHeading(270))
                    .strafeToSplineHeading(pos(36, -27), degreeHeading(270))
                    //--- Turn on intake
                    .stopAndAdd(new AutoActions.IntakeOn(robot))
                    //--- Drive forward to pick up balls at half speed
                    .strafeToSplineHeading(pos(36, -40), degreeHeading(270), verySlow(), slowAccel())
                    //--- Return to shoot position (2)
                    .strafeToSplineHeading(pos(-12, -12), degreeHeading(225))
                    //--- Stop intake
                    .stopAndAdd(new AutoActions.IntakeStop(robot))
                    //--- Fire stack balls
                    .stopAndAdd(getFireAction("Stack 1"))
                    .build()
            );
        }

        //========================================================================
        //--- PARK
        //========================================================================
        if (selectedPark == ParkPosition.PARK)
        {
            Pose2d currentPose = (selectedStacks == StackSelection.STACK_1) 
                ? pose(-12, -12, 225) 
                : pose(-12, -12, 225);
                
            Actions.runBlocking(
                drive.actionBuilder(currentPose)
                    //--- Drive to park position (12, -12)
                    .strafeToSplineHeading(pos(12, -12), degreeHeading(270))
                    //--- Stop intake
                    .stopAndAdd(new AutoActions.IntakeStop(robot))
                    .build()
            );
        }

        //--- Stop flywheel and intake
        Actions.runBlocking(new AutoActions.FlywheelStop(robot));
        Actions.runBlocking(new AutoActions.IntakeStop(robot));
        
        //--- Log completion time
        Actions.runBlocking(new AutoActions.LogMessage("DONE", fireLog, autoTimer));
    }

    //========================================================================
    //--- RED ALLIANCE ROUTINE (mirrored from Blue)
    //========================================================================
    private void runRedAlliance() 
    {
        //--- Starting position (1) - Far side for Red (mirrored from Blue)
        //--- Blue: (60, -12, 180°) → Red: (60, 12, 180°)
        Pose2d startPose = pose(60, 12, 180);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        //========================================================================
        //--- Drive to shoot position (2) and fire preloads
        //========================================================================
        Actions.runBlocking(
            drive.actionBuilder(startPose)
                //--- Start flywheel while driving
                .stopAndAdd(new AutoActions.FlywheelSetSpeed(robot, SHOOT_3_RPM-50))
                //--- Programmable delay
                .waitSeconds(startDelaySeconds)
                //--- Drive to shoot position (2) - pointing at red goal
                //--- Blue: (-12, -12, 225°) → Red: (-12, 12, 135°)
                .strafeToSplineHeading(pos(-12, 12), degreeHeading(135))
                //--- Give flywheel time to spin up
                .waitSeconds(1.0)
                //--- Fire preloads
                .stopAndAdd(getFireAction("Preloads"))
                .build()
        );

        //========================================================================
        //--- Stack 1 (far stack at X=36) - if selected
        //========================================================================
        if (selectedStacks == StackSelection.STACK_1)
        {
            Actions.runBlocking(
                drive.actionBuilder(pose(-12, 12, 135))
                    //--- Align with balls (Stack at X=36, mirrored Y)
                    //--- Blue: (36, -16 to -40, 270°) → Red: (36, 16 to 40, 90°)
                    .strafeToSplineHeading(pos(36, 16), degreeHeading(90))
                    .strafeToSplineHeading(pos(36, 27), degreeHeading(90))
                    //--- Turn on intake
                    .stopAndAdd(new AutoActions.IntakeOn(robot))
                    //--- Drive forward to pick up balls at half speed
                    .strafeToSplineHeading(pos(36, 40), degreeHeading(90), verySlow(), slowAccel())
                    //--- Return to shoot position (2)
                    //--- Blue: (-12, -12, 225°) → Red: (-12, 12, 135°)
                    .strafeToSplineHeading(pos(-12, 12), degreeHeading(135))
                    //--- Stop intake
                    .stopAndAdd(new AutoActions.IntakeStop(robot))
                    //--- Fire stack balls
                    .stopAndAdd(getFireAction("Stack 1"))
                    .build()
            );
        }

        //========================================================================
        //--- PARK
        //========================================================================
        if (selectedPark == ParkPosition.PARK)
        {
            Pose2d currentPose = (selectedStacks == StackSelection.STACK_1) 
                ? pose(-12, 12, 135) 
                : pose(-12, 12, 135);
                
            Actions.runBlocking(
                drive.actionBuilder(currentPose)
                    //--- Drive to park position (12, 12) - mirrored Y
                    //--- Blue: (12, -12, 270°) → Red: (12, 12, 90°)
                    .strafeToSplineHeading(pos(12, 12), degreeHeading(90))
                    //--- Stop intake
                    .stopAndAdd(new AutoActions.IntakeStop(robot))
                    .build()
            );
        }

        //--- Stop flywheel and intake
        Actions.runBlocking(new AutoActions.FlywheelStop(robot));
        Actions.runBlocking(new AutoActions.IntakeStop(robot));
        
        //--- Log completion time
        Actions.runBlocking(new AutoActions.LogMessage("DONE", fireLog, autoTimer));
    }

    //========================================================================
    //--- BALL COLOR LIGHT DISPLAY
    //========================================================================
    /**
     * Shows the detected ball colors on the lights (left/middle/right).
     * Green balls = Green light, Purple balls = Purple light, None = Off
     */
    private void showBallColorsOnLights()
    {
        //--- Get ball colors from intake sensors
        org.firstinspires.ftc.teamcode.hardware.Intake.BallColor leftBall = robot.intake.getLeftBallColor();
        org.firstinspires.ftc.teamcode.hardware.Intake.BallColor centerBall = robot.intake.getCenterBallColor();
        org.firstinspires.ftc.teamcode.hardware.Intake.BallColor rightBall = robot.intake.getRightBallColor();
        
        //--- Set each light based on ball color
        robot.lights.setLeft(ballColorToLightColor(leftBall));
        robot.lights.setMiddle(ballColorToLightColor(centerBall));
        robot.lights.setRight(ballColorToLightColor(rightBall));
    }
    
    /**
     * Converts an Intake.BallColor to a Lights.Color
     */
    private Lights.Color ballColorToLightColor(org.firstinspires.ftc.teamcode.hardware.Intake.BallColor ballColor)
    {
        switch (ballColor)
        {
            case GREEN:
                return Lights.Color.GREEN;
            case PURPLE:
                return Lights.Color.PURPLE;
            case UNKNOWN:
                return Lights.Color.YELLOW;  // Unknown = Yellow to indicate issue
            case NONE:
            default:
                return Lights.Color.OFF;
        }
    }

    //========================================================================
    //--- FIRING ACTION HELPER
    //========================================================================
    /**
     * Returns the appropriate firing action based on the selected firing mode.
     * SEQUENCE: Fires balls one at a time based on detected color (green first)
     * ALL: Fires all balls at once (faster but no color sorting)
     */
    private Action getFireAction(String label)
    {
        if (selectedFiringMode == FiringMode.SEQUENCE)
        {
            return new AutoActions.KickerWaitForSpeedThenFireSequence(robot, SHOOT_1_RPM, label, fireLog, autoTimer);
        }
        else
        {
            return new AutoActions.KickerWaitForSpeedThenFireAll(robot, SHOOT_1_RPM, SHOOT_2_RPM, SHOOT_3_RPM, label, fireLog);
        }
    }

}
