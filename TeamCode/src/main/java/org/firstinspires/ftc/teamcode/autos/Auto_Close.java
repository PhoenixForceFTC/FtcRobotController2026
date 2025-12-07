package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Camera;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.utils.AutoUtils.pos;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.degreeHeading;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.pose;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.slow;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.slowAccel;

/**
 * Auto_Close
 * 
 * Full autonomous routine with ALLIANCE SELECTION:
 * - Press X on gamepad2 during init to select BLUE alliance
 * - Press B on gamepad2 during init to select RED alliance
 * - Press Y on gamepad2 to select PARK IN FRONT
 * - Press A on gamepad2 to select PARK BY LEVER (default)
 * - Press Dpad Up on gamepad2 to increase start delay (+1 second)
 * - Press Dpad Down on gamepad2 to decrease start delay (-1 second)
 * 
 * Blue Alliance: Starts at (-55, -45) facing 235°
 * Red Alliance:  Starts at (+55, -45) facing 305° (mirrored)
 * 
 * Routine:
 * 1. Shoots preloaded balls (1-3)
 * 2. Picks up balls from first stack, shoots (4-6)
 * 3. Opens second stack, picks up, shoots (7-9)
 * 4. Picks up third stack, parks
 */
@Autonomous(name = "Auto: Close", group = "Auto")
public class Auto_Close extends LinearOpMode {

    //--- Robot hardware
    private RobotHardware robot;
    
    //--- Flywheel speeds for shooting (RPM)
    //--- When firing all 3 at once, flywheel slows down so we need higher initial RPM
    //--- When firing 1 at a time (sequence), each ball uses the single-ball RPM
    private static final double SHOOT_RPM_1_BALL = 2000.0;   // Single ball / sequence firing
    private static final double SHOOT_RPM_2_BALLS = 2300.0;  // Two balls at once
    private static final double SHOOT_RPM_3_BALLS = 2600.0;  // Three balls at once (fire all)

    //--- Pre-match camera positions for sequence detection (tune based on starting position)
    //--- Yaw: 0.0 = full right, 0.5 = center, 1.0 = full left
    //--- Pitch: 0.0 = down, 0.5 = level, 1.0 = up
    private static final double PREMATCH_BLUE_YAW = 0.35;    // Blue: look left toward obelisk
    private static final double PREMATCH_BLUE_PITCH = 0.70;
    private static final double PREMATCH_RED_YAW = 0.65;     // Red: look right toward obelisk
    private static final double PREMATCH_RED_PITCH = 0.70;
    
    //--- Log of firing speeds for each phase
    private List<String> fireLog = new ArrayList<>();

    //--- Alliance selection
    private enum Alliance { BLUE, RED }
    private Alliance selectedAlliance = Alliance.BLUE;  // Default to blue

    //--- Parking position selection
    private enum ParkPosition { LEVER, FRONT }
    private ParkPosition selectedPark = ParkPosition.LEVER;  // Default to lever

    //--- Detected ball sequence
    private Camera.BallSequence detectedSequence = Camera.BallSequence.UNKNOWN;

    //--- Programmable delay (seconds) - adjustable with dpad up/down
    private int startDelaySeconds = 0;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    @Override
    public void runOpMode() throws InterruptedException 
    {
        //--- Initialize robot hardware
        robot = new RobotHardware(this);
        robot.init(2);  // Beta robot

        //========================================================================
        //--- Alliance Selection Loop (during init, before start)
        //========================================================================
        //--- Set initial camera position for Blue alliance
        robot.camera.setPreMatchPosition(PREMATCH_BLUE_YAW, PREMATCH_BLUE_PITCH);
        
        while (!isStarted() && !isStopRequested()) 
        {
            //--- Check for alliance selection buttons on gamepad2
            if (gamepad2.x) 
            {
                if (selectedAlliance != Alliance.BLUE)
                {
                    selectedAlliance = Alliance.BLUE;
                    robot.lights.setAllBlue();
                    robot.camera.setPreMatchPosition(PREMATCH_BLUE_YAW, PREMATCH_BLUE_PITCH);
                }
            } 
            else if (gamepad2.b) 
            {
                if (selectedAlliance != Alliance.RED)
                {
                    selectedAlliance = Alliance.RED;
                    robot.lights.setAllRed();
                    robot.camera.setPreMatchPosition(PREMATCH_RED_YAW, PREMATCH_RED_PITCH);
                }
            }

            //--- Run pre-match detection (updates lights when sequence detected)
            detectedSequence = robot.camera.runPreMatchDetection();

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

            //--- Display current selection and detection status
            telemetry.addData("=== ALLIANCE SELECTION ===", "");
            telemetry.addData("Press X", "BLUE Alliance");
            telemetry.addData("Press B", "RED Alliance");
            telemetry.addLine("");
            telemetry.addData(">>> SELECTED", selectedAlliance);
            telemetry.addLine("");
            telemetry.addData("=== PARKING POSITION ===", "");
            telemetry.addData("Press Y", "Park IN FRONT");
            telemetry.addData("Press A", "Park BY LEVER");
            telemetry.addLine("");
            telemetry.addData(">>> PARK", selectedPark);
            telemetry.addLine("");
            telemetry.addData("=== START DELAY ===", "");
            telemetry.addData("Dpad Up/Down", "+/- 1 second");
            telemetry.addData(">>> DELAY", "%d seconds", startDelaySeconds);
            telemetry.addLine("");
            telemetry.addData("=== SEQUENCE DETECTION ===", "");
            telemetry.addData("Camera Connected", robot.camera.isConnected());
            telemetry.addData("Last Tag", robot.camera.getLastDetectedTag());
            telemetry.addData(">>> SEQUENCE", detectedSequence);
            telemetry.addLine("");
            telemetry.addData("Status", "Waiting for START...");
            telemetry.update();
        }

        if (isStopRequested()) return;

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
        //--- Starting position (Blue side)
        Pose2d startPose = pose(-55, -45, 235);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        //========================================================================
        //--- Preloads, shoot (1-3)
        //========================================================================
        Actions.runBlocking(
            drive.actionBuilder(startPose)
                //--- Start flywheel while driving
                .stopAndAdd(new AutoActions.FlywheelSetSpeed(robot, SHOOT_RPM_1_BALL))
                .waitSeconds(2.0)  //--- Give flywheel time to spin up
                //--- Programmable delay (set during init with dpad up/down)
                .waitSeconds(startDelaySeconds)
                //--- Align to shoot
                .strafeToSplineHeading(pos(-33.5, -36.5), degreeHeading(227))
                //--- Wait for flywheel to get up to speed, then fire in sequence
                .stopAndAdd(new AutoActions.KickerWaitForSpeedThenFireSequence(robot, SHOOT_RPM_1_BALL, "Preloads", fireLog))
                .build()
        );

        //========================================================================
        //--- Stack 1, shoot (4-6)
        //========================================================================
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
                //--- Wait for flywheel to get up to speed, then fire in sequence
                .stopAndAdd(new AutoActions.KickerWaitForSpeedThenFireSequence(robot, SHOOT_RPM_1_BALL, "Stack 1", fireLog))
                .build()
        );

        //========================================================================
        //--- Stack 2, shoot (7-9)
        //========================================================================
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
                //--- Wait for flywheel to get up to speed, then fire in sequence
                .stopAndAdd(new AutoActions.KickerWaitForSpeedThenFireSequence(robot, SHOOT_RPM_1_BALL, "Stack 2", fireLog))
                .build()
        );

        //========================================================================
        //--- Stack 3 --- collect but don't shoot
        //========================================================================
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

        //========================================================================
        //--- PARK based on selection
        //========================================================================
        if (selectedPark == ParkPosition.LEVER)
        {
            //--- PARK next to lever
            Actions.runBlocking(
                drive.actionBuilder(pose(30, -50, 0)) //--- Assumed pose after previous action
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
                drive.actionBuilder(pose(30, -50, 0)) //--- Assumed pose after previous action
                    //--- Drive to front parking position
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
    //--- RED ALLIANCE ROUTINE
    //========================================================================
    private void runRedAlliance() 
    {
        //--- Starting position (Red side - mirrored)
        //--- Blue: (-55, -45, 235°) → Red: (+55, -45, 305°)
        Pose2d startPose = pose(55, -45, 305);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        //========================================================================
        //--- Preloads, shoot (1-3)
        //========================================================================
        Actions.runBlocking(
            drive.actionBuilder(startPose)
                .stopAndAdd(new AutoActions.FlywheelSetSpeed(robot, SHOOT_RPM_1_BALL))
                .waitSeconds(2.0)
                //--- Programmable delay (set during init with dpad up/down)
                .waitSeconds(startDelaySeconds)
                //--- Blue: (-33.5, -36.5, 227°) → Red: (+33.5, -36.5, 313°)
                .strafeToSplineHeading(pos(33.5, -36.5), degreeHeading(313))
                //--- Wait for flywheel to get up to speed, then fire in sequence
                .stopAndAdd(new AutoActions.KickerWaitForSpeedThenFireSequence(robot, SHOOT_RPM_1_BALL, "Preloads", fireLog))
                .build()
        );

        //========================================================================
        //--- Stack 1, shoot (4-6)
        //========================================================================
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
                //--- Wait for flywheel to get up to speed, then fire in sequence
                .stopAndAdd(new AutoActions.KickerWaitForSpeedThenFireSequence(robot, SHOOT_RPM_1_BALL, "Stack 1", fireLog))
                .build()
        );

        //========================================================================
        //--- Stack 2, shoot (7-9)
        //========================================================================
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
                //--- Wait for flywheel to get up to speed, then fire in sequence
                .stopAndAdd(new AutoActions.KickerWaitForSpeedThenFireSequence(robot, SHOOT_RPM_1_BALL, "Stack 2", fireLog))
                .build()
        );

        //========================================================================
        //--- Stack 3 --- collect but don't shoot
        //========================================================================
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

        //========================================================================
        //--- PARK based on selection
        //========================================================================
        if (selectedPark == ParkPosition.LEVER)
        {
            //--- PARK next to lever
            Actions.runBlocking(
                drive.actionBuilder(pose(-30, -50, 180)) //--- Assumed pose after previous action
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
                drive.actionBuilder(pose(-30, -50, 180)) //--- Assumed pose after previous action
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
}
