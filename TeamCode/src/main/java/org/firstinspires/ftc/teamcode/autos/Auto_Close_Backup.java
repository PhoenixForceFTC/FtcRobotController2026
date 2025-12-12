package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.slow;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.slowAccel;

/**
 * Auto_Close
 * 
 * Full autonomous routine with ALLIANCE SELECTION:
 * - Press X on gamepad2 during init to select BLUE alliance (default)
 * - Press B on gamepad2 during init to select RED alliance
 * - Press Y on gamepad2 to select PARK IN FRONT
 * - Press A on gamepad2 to select PARK BY LEVER (default)
 * - Press Dpad Left/Right to select how many stacks to collect (default: Stack 3)
 * - Press Dpad Up on gamepad2 to increase start delay (+1 second)
 * - Press Dpad Down on gamepad2 to decrease start delay (-1 second)
 * 
 * Blue Alliance: Starts at (-55, -45) facing 55° (toward obelisk for AprilTag detection)
 * Red Alliance:  Starts at (+55, -45) facing 125° (mirrored, toward obelisk)
 * 
 * Routine:
 * 1. Shoots preloaded balls (1-3)
 * 2. Picks up balls from first stack, shoots (4-6)
 * 3. Opens second stack, picks up, shoots (7-9)
 * 4. Picks up third stack, parks
 */
@Disabled
@Autonomous(name = "Auto: Close", group = "Auto")
public class Auto_Close_Backup extends LinearOpMode 
{
    //--- Robot hardware
    private RobotHardware robot;
    
    //--- Flywheel speed for shooting (RPM)
    //--- Firing in sequence (one ball at a time) uses single-ball RPM
    private static final double SHOOT_RPM = 2000.0; // 2350

    //--- Pre-match camera positions for sequence detection (tune based on starting position)
    //--- Yaw: 0.0 = full right, 0.5 = center, 1.0 = full left
    //--- Pitch: 0.0 = down, 0.5 = level, 1.0 = up
    private static final double PREMATCH_BLUE_YAW = 0.6;    // Blue: look left toward obelisk
    private static final double PREMATCH_BLUE_PITCH = 0.60;
    private static final double PREMATCH_RED_YAW = 0.4;     // Red: look right toward obelisk
    private static final double PREMATCH_RED_PITCH = 0.60;
    
    //--- Log of firing speeds for each phase
    private List<String> fireLog = new ArrayList<>();

    //--- Alliance selection
    private enum Alliance { BLUE, RED }
    private Alliance selectedAlliance = Alliance.BLUE;  // Default to blue

    //--- Parking position selection
    private enum ParkPosition { LEVER, FRONT }
    //private ParkPosition selectedPark = ParkPosition.LEVER;  // Default to lever
    private ParkPosition selectedPark = ParkPosition.FRONT;  // Default to lever

    //--- Stack selection (how many stacks to collect before parking)
    private enum StackSelection { PRELOADS_ONLY, STACK_1, STACK_2, STACK_3 }
    //private StackSelection selectedStacks = StackSelection.STACK_3;  // Default to all stacks
    private StackSelection selectedStacks = StackSelection.PRELOADS_ONLY;
    private boolean dpadLeftPressed = false;
    private boolean dpadRightPressed = false;

    //--- Detected ball sequence
    private Camera.BallSequence detectedSequence = Camera.BallSequence.UNKNOWN;

    //--- Programmable delay (seconds) - adjustable with dpad up/down
    private int startDelaySeconds = 0;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    //--- Light timer for pre-match display (show alliance color, then let camera take over)
    private ElapsedTime lightTimer = new ElapsedTime();
    private static final double ALLIANCE_DISPLAY_TIME = 3.0;  // Seconds to show alliance color
    private boolean allianceDisplayComplete = false;  // Flag to track if initial display is done

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
        lightTimer.reset();
        allianceDisplayComplete = false;

        //========================================================================
        //--- Alliance Selection Loop (during init, before start)
        //========================================================================
        //--- Set initial camera position based on default alliance (Blue)
        if (selectedAlliance == Alliance.BLUE)
        {
            robot.camera.setPreMatchPosition(PREMATCH_BLUE_YAW, PREMATCH_BLUE_PITCH);
        }
        else
        {
            robot.camera.setPreMatchPosition(PREMATCH_RED_YAW, PREMATCH_RED_PITCH);
        }
        
        while (!isStarted() && !isStopRequested()) 
        {
            //--- Check for alliance selection buttons on gamepad2
            if (gamepad2.x) 
            {
                if (selectedAlliance != Alliance.BLUE)
                {
                    selectedAlliance = Alliance.BLUE;
                    robot.camera.setPreMatchPosition(PREMATCH_BLUE_YAW, PREMATCH_BLUE_PITCH);
                    //--- Reset light display to show new alliance color
                    robot.lights.setAllBlue();
                    lightTimer.reset();
                    allianceDisplayComplete = false;
                }
            } 
            else if (gamepad2.b) 
            {
                if (selectedAlliance != Alliance.RED)
                {
                    selectedAlliance = Alliance.RED;
                    robot.camera.setPreMatchPosition(PREMATCH_RED_YAW, PREMATCH_RED_PITCH);
                    //--- Reset light display to show new alliance color
                    robot.lights.setAllRed();
                    lightTimer.reset();
                    allianceDisplayComplete = false;
                }
            }

            //--- Update pre-match lights (handles alliance display timing)
            //--- Only run camera detection after alliance display is complete
            updatePreMatchLights();
            
            if (allianceDisplayComplete)
            {
                //--- Run pre-match detection (updates lights when sequence detected)
                detectedSequence = robot.camera.runPreMatchDetection();
            }

            //--- Check for parking position selection on gamepad2
            if (gamepad2.y)
            {
                selectedPark = ParkPosition.FRONT;
            }
            else if (gamepad2.a)
            {
                selectedPark = ParkPosition.LEVER;
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
                    //--- Cycle down: STACK_3 -> STACK_2 -> STACK_1 -> PRELOADS_ONLY
                    switch (selectedStacks)
                    {
                        case STACK_3: selectedStacks = StackSelection.STACK_2; break;
                        case STACK_2: selectedStacks = StackSelection.STACK_1; break;
                        case STACK_1: selectedStacks = StackSelection.PRELOADS_ONLY; break;
                        case PRELOADS_ONLY: break; // Already at minimum
                    }
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
                    //--- Cycle up: PRELOADS_ONLY -> STACK_1 -> STACK_2 -> STACK_3
                    switch (selectedStacks)
                    {
                        case PRELOADS_ONLY: selectedStacks = StackSelection.STACK_1; break;
                        case STACK_1: selectedStacks = StackSelection.STACK_2; break;
                        case STACK_2: selectedStacks = StackSelection.STACK_3; break;
                        case STACK_3: break; // Already at maximum
                    }
                }
            }
            else
            {
                dpadRightPressed = false;
            }

            //--- Display current selection and detection status
            telemetry.addData("=== ALLIANCE SELECTION ===", "");
            telemetry.addData("Press", "X=BLUE, B=RED");
            telemetry.addData(">>> SELECTED", selectedAlliance);
            //telemetry.addLine("");
            telemetry.addData("=== PARKING POSITION ===", "");
            telemetry.addData("Press", "Y=IN FRONT, A=BY LEVER");
            telemetry.addData(">>> PARK", selectedPark);
            //telemetry.addLine("");
            telemetry.addData("=== STACK SELECTION ===", "");
            telemetry.addData("Dpad Left/Right", "Fewer/More stacks");
            telemetry.addData(">>> COLLECT", selectedStacks);
            //telemetry.addLine("");
            telemetry.addData("=== START DELAY ===", "");
            telemetry.addData("Dpad Up/Down", "+/- 1 second");
            telemetry.addData(">>> DELAY", "%d seconds", startDelaySeconds);
            //telemetry.addLine("");
            telemetry.addData("=== SEQUENCE DETECTION ===", "");
            telemetry.addData("Camera Connected", robot.camera.isConnected());
            telemetry.addData("Last Tag", robot.camera.getLastDetectedTag());
            telemetry.addData(">>> SEQUENCE", detectedSequence);
            telemetry.addLine("");
            telemetry.addData("Status", "Waiting for START...");
            telemetry.update();
        }

        if (isStopRequested()) return;

        //--- Re-enable mode system for autonomous (intake will control lights)
        robot.lights.setModeEnabled(true);
        robot.lights.forceMode(Lights.LightMode.DEFAULT);

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
        telemetry.addData("Sequence", detectedSequence);
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
        //--- Starting position (Blue side) - facing toward obelisk for AprilTag detection
        //Pose2d startPose = pose(-55, -45, 55);

        Pose2d startPose = pose(-38, -53, 90);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        //========================================================================
        //--- Preloads, shoot (1-3)
        //========================================================================
        Actions.runBlocking(
            drive.actionBuilder(startPose)
                //--- Start flywheel while driving
                .stopAndAdd(new AutoActions.FlywheelSetSpeed(robot, SHOOT_RPM))
                .waitSeconds(2.0)  //--- Give flywheel time to spin up
                //--- Programmable delay (set during init with dpad up/down)
                .waitSeconds(startDelaySeconds)
                //--- Align to shoot
                //.strafeToSplineHeading(pos(-33, -31), degreeHeading(227))
                .strafeToSplineHeading(pos(-20, -20), degreeHeading(227))
                //--- Switch camera to TeleOp mode (target detection)
                .stopAndAdd(new AutoActions.CameraSetTeleOpMode(robot))
                //--- Auto-aim and fire in sequence
                .stopAndAdd(new AutoActions.AutoAimAndFireSequence(robot, "Preloads", fireLog))
                .build()
        );

        //========================================================================
        //--- Stack 1, shoot (4-6) - if selected
        //========================================================================
        if (selectedStacks == StackSelection.STACK_1 || 
            selectedStacks == StackSelection.STACK_2 || 
            selectedStacks == StackSelection.STACK_3)
        {
            Actions.runBlocking(
                drive.actionBuilder(pose(-36, -36, 235))
                    //--- Align with balls
                    .strafeToSplineHeading(pos(-32, -50), degreeHeading(10))
                    //--- Turn on intake
                    .stopAndAdd(new AutoActions.IntakeOn(robot))
                    //--- Drive forward to pick up balls at half speed
                    .strafeToSplineHeading(pos(-16, -50), degreeHeading(0), slow(), slowAccel())
                    //--- Reverse intake while moving to shoot position
                    //.stopAndAdd(new AutoActions.IntakeReverse(robot))
                    //--- Align to shoot
                    .strafeToSplineHeading(pos(-36, -36), degreeHeading(220))
                    //--- Stop intake
                    .stopAndAdd(new AutoActions.IntakeStop(robot))
                    //--- Auto-aim and fire in sequence
                    .stopAndAdd(new AutoActions.AutoAimAndFireSequence(robot, "Stack 1", fireLog))
                    .build()
            );
        }

        //========================================================================
        //--- Stack 2, shoot (7-9) - if selected
        //========================================================================
        if (selectedStacks == StackSelection.STACK_2 || 
            selectedStacks == StackSelection.STACK_3)
        {
            Actions.runBlocking(
                drive.actionBuilder(pose(-36, -36, 235))
                    //--- Align with balls
                    .strafeToSplineHeading(pos(-9, -50), degreeHeading(10))
                    //--- Turn on intake
                    .stopAndAdd(new AutoActions.IntakeOn(robot))
                    //--- Drive forward to pick up balls at half speed
                    .strafeToSplineHeading(pos(7, -50), degreeHeading(0), slow(), slowAccel())
                    //--- Reverse intake while moving to shoot position
                    //.stopAndAdd(new AutoActions.IntakeReverse(robot))
                    //--- Align to shoot
                    .strafeToSplineHeading(pos(-36, -36), degreeHeading(224))
                    //--- Stop intake
                    .stopAndAdd(new AutoActions.IntakeStop(robot))
                    //--- Auto-aim and fire in sequence
                    .stopAndAdd(new AutoActions.AutoAimAndFireSequence(robot, "Stack 2", fireLog))
                    .build()
            );
        }

        //========================================================================
        //--- Stack 3 --- collect but don't shoot - if selected
        //========================================================================
        if (selectedStacks == StackSelection.STACK_3)
        {
            Actions.runBlocking(
                drive.actionBuilder(pose(-36, -36, 235))
                    //--- Align with balls
                    .strafeToSplineHeading(pos(14, -50), degreeHeading(10))
                    //--- Turn on intake
                    .stopAndAdd(new AutoActions.IntakeOn(robot))
                    //--- Drive forward to pick up balls at half speed
                    .strafeToSplineHeading(pos(30, -50), degreeHeading(0), slow(), slowAccel())
                    .build()
            );
        }

        //========================================================================
        //--- PARK based on selection
        //========================================================================
        //--- Determine starting pose for parking based on which stacks were collected
        //--- Stack 3: ends at (30, -50, 0°)
        //--- Preloads/Stack 1/Stack 2: ends at (-36, -36, 235°)
        Pose2d parkStartPose = (selectedStacks == StackSelection.STACK_3) 
            ? pose(30, -50, 0) 
            : pose(-36, -36, 235);

        if (selectedPark == ParkPosition.LEVER)
        {
            //--- PARK next to lever
            Actions.runBlocking(
                drive.actionBuilder(parkStartPose)
                    //--- Align to lever position
                    .strafeToSplineHeading(pos(0, -50), degreeHeading(180))
                    //--- Stop intake
                    .stopAndAdd(new AutoActions.IntakeStop(robot))
                    .build()
            );
        }
        else
        {
            //--- PARK in front (facing the goal)
            Actions.runBlocking(
                drive.actionBuilder(parkStartPose)
                    //--- Drive to front parking position
                    .strafeToSplineHeading(pos(-50, -20), degreeHeading(270))
                    //--- Stop intake
                    .stopAndAdd(new AutoActions.IntakeStop(robot))
                    .build()
            );
        }

        //--- Stop flywheel and intake
        Actions.runBlocking(new AutoActions.FlywheelStop(robot));
        Actions.runBlocking(new AutoActions.IntakeStop(robot));
    }

    //========================================================================
    //--- RED ALLIANCE ROUTINE
    //========================================================================
    private void runRedAlliance() 
    {
        //--- Starting position (Red side - mirrored) - facing toward obelisk for AprilTag detection
        //--- Blue: (-55, -45, 55°) → Red: (+55, -45, 125°)
        Pose2d startPose = pose(55, -45, 125);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        //========================================================================
        //--- Preloads, shoot (1-3)
        //========================================================================
        Actions.runBlocking(
            drive.actionBuilder(startPose)
                .stopAndAdd(new AutoActions.FlywheelSetSpeed(robot, SHOOT_RPM))
                .waitSeconds(2.0)
                //--- Programmable delay (set during init with dpad up/down)
                .waitSeconds(startDelaySeconds)
                //--- Blue: (-33.5, -36.5, 227°) → Red: (+33.5, -36.5, 313°)
                .strafeToSplineHeading(pos(33.5, -36.5), degreeHeading(313))
                //--- Switch camera to TeleOp mode (target detection)
                .stopAndAdd(new AutoActions.CameraSetTeleOpMode(robot))
                //--- Auto-aim and fire in sequence
                .stopAndAdd(new AutoActions.AutoAimAndFireSequence(robot, "Preloads", fireLog))
                .build()
        );

        //========================================================================
        //--- Stack 1, shoot (4-6) - if selected
        //========================================================================
        if (selectedStacks == StackSelection.STACK_1 || 
            selectedStacks == StackSelection.STACK_2 || 
            selectedStacks == StackSelection.STACK_3)
        {
            Actions.runBlocking(
                drive.actionBuilder(pose(36, -36, 305))
                    //--- Align with balls
                    //--- Blue: (-32, -50, 10°) → Red: (+32, -50, 170°)
                    .strafeToSplineHeading(pos(32, -50), degreeHeading(170))
                    //--- Turn on intake
                    .stopAndAdd(new AutoActions.IntakeOn(robot))
                    //--- Drive forward to pick up balls at half speed
                    //--- Blue: (-16, -50, 0°) → Red: (+16, -50, 180°)
                    .strafeToSplineHeading(pos(16, -50), degreeHeading(180), slow(), slowAccel())
                    //--- Reverse intake while moving to shoot position
                    //.stopAndAdd(new AutoActions.IntakeReverse(robot))
                    //--- Align to shoot
                    //--- Blue: (-36, -36, 220°) → Red: (+36, -36, 320°)
                    .strafeToSplineHeading(pos(36, -36), degreeHeading(320))
                    //--- Stop intake
                    .stopAndAdd(new AutoActions.IntakeStop(robot))
                    //--- Auto-aim and fire in sequence
                    .stopAndAdd(new AutoActions.AutoAimAndFireSequence(robot, "Stack 1", fireLog))
                    .build()
            );
        }

        //========================================================================
        //--- Stack 2, shoot (7-9) - if selected
        //========================================================================
        if (selectedStacks == StackSelection.STACK_2 || 
            selectedStacks == StackSelection.STACK_3)
        {
            Actions.runBlocking(
                drive.actionBuilder(pose(36, -36, 305))
                    //--- Align with balls
                    //--- Blue: (-9, -50, 10°) → Red: (+9, -50, 170°)
                    .strafeToSplineHeading(pos(9, -50), degreeHeading(170))
                    //--- Turn on intake
                    .stopAndAdd(new AutoActions.IntakeOn(robot))
                    //--- Drive forward to pick up balls at half speed
                    //--- Blue: (7, -50, 0°) → Red: (-7, -50, 180°)
                    .strafeToSplineHeading(pos(-7, -50), degreeHeading(180), slow(), slowAccel())
                    //--- Reverse intake while moving to shoot position
                    //.stopAndAdd(new AutoActions.IntakeReverse(robot))
                    //--- Align to shoot
                    //--- Blue: (-36, -36, 224°) → Red: (+36, -36, 316°)
                    .strafeToSplineHeading(pos(36, -36), degreeHeading(316))
                    //--- Stop intake
                    .stopAndAdd(new AutoActions.IntakeStop(robot))
                    //--- Auto-aim and fire in sequence
                    .stopAndAdd(new AutoActions.AutoAimAndFireSequence(robot, "Stack 2", fireLog))
                    .build()
            );
        }

        //========================================================================
        //--- Stack 3 --- collect but don't shoot - if selected
        //========================================================================
        if (selectedStacks == StackSelection.STACK_3)
        {
            Actions.runBlocking(
                drive.actionBuilder(pose(36, -36, 305))
                    //--- Align with balls
                    //--- Blue: (14, -50, 10°) → Red: (-14, -50, 170°)
                    .strafeToSplineHeading(pos(-14, -50), degreeHeading(170))
                    //--- Turn on intake
                    .stopAndAdd(new AutoActions.IntakeOn(robot))
                    //--- Drive forward to pick up balls at half speed
                    //--- Blue: (30, -50, 0°) → Red: (-30, -50, 180°)
                    .strafeToSplineHeading(pos(-30, -50), degreeHeading(180), slow(), slowAccel())
                    .build()
            );
        }

        //========================================================================
        //--- PARK based on selection
        //========================================================================
        //--- Determine starting pose for parking based on which stacks were collected
        //--- Stack 3: ends at (-30, -50, 180°)
        //--- Preloads/Stack 1/Stack 2: ends at (36, -36, 305°)
        Pose2d parkStartPose = (selectedStacks == StackSelection.STACK_3) 
            ? pose(-30, -50, 180) 
            : pose(36, -36, 305);

        if (selectedPark == ParkPosition.LEVER)
        {
            //--- PARK next to lever
            Actions.runBlocking(
                drive.actionBuilder(parkStartPose)
                    //--- Align to lever position
                    //--- Blue: (0, -50, 180°) → Red: (0, -50, 0°)
                    .strafeToSplineHeading(pos(0, -50), degreeHeading(0))
                    //--- Stop intake
                    .stopAndAdd(new AutoActions.IntakeStop(robot))
                    .build()
            );
        }
        else
        {
            //--- PARK in front (facing the goal)
            Actions.runBlocking(
                drive.actionBuilder(parkStartPose)
                    //--- Drive to front parking position
                    //--- Blue: (0, -36, 270°) → Red: (0, -36, 270°) (same position, facing goal)
                    .strafeToSplineHeading(pos(0, -36), degreeHeading(270))
                    //--- Stop intake
                    .stopAndAdd(new AutoActions.IntakeStop(robot))
                    .build()
            );
        }

        //--- Stop flywheel and intake
        Actions.runBlocking(new AutoActions.FlywheelStop(robot));
        Actions.runBlocking(new AutoActions.IntakeStop(robot));
    }

    //========================================================================
    //--- PRE-MATCH LIGHT DISPLAY
    //========================================================================
    /**
     * Updates the lights during pre-match:
     * - First 3 seconds: Show alliance color (Blue/Red)
     * - After 3 seconds: Let camera detection control lights (shows GPP/PGP/PPG pattern)
     */
    private void updatePreMatchLights()
    {
        //--- During initial alliance display period, keep showing alliance color
        if (!allianceDisplayComplete)
        {
            if (lightTimer.seconds() >= ALLIANCE_DISPLAY_TIME)
            {
                allianceDisplayComplete = true;
                //--- Camera's runPreMatchDetection() will now control the lights
            }
            //--- Don't change lights during alliance display period
            return;
        }
        
        //--- After alliance display, camera.runPreMatchDetection() handles the lights
        //--- (it sets lights based on detected sequence: GPP/PGP/PPG)
    }

}
