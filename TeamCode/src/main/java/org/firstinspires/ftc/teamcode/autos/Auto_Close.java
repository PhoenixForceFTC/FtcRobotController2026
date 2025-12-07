package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;

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
    
    //--- Flywheel speed for shooting
    private static final double SHOOT_RPM = 2600.0;
    
    //--- Log of firing speeds for each phase
    private List<String> fireLog = new ArrayList<>();

    //--- Alliance selection
    private enum Alliance { BLUE, RED }
    private Alliance selectedAlliance = Alliance.BLUE;  // Default to blue

    @Override
    public void runOpMode() throws InterruptedException 
    {
        //--- Initialize robot hardware
        robot = new RobotHardware(this);
        robot.init(2);  // Beta robot

        //========================================================================
        //--- Alliance Selection Loop (during init, before start)
        //========================================================================
        while (!isStarted() && !isStopRequested()) 
        {
            //--- Check for alliance selection buttons on gamepad2
            if (gamepad2.x) 
            {
                selectedAlliance = Alliance.BLUE;
                robot.lights.setAllBlue();
            } 
            else if (gamepad2.b) 
            {
                selectedAlliance = Alliance.RED;
                robot.lights.setAllRed();
            }

            //--- Display current selection
            telemetry.addData("=== ALLIANCE SELECTION ===", "");
            telemetry.addData("Press X", "BLUE Alliance");
            telemetry.addData("Press B", "RED Alliance");
            telemetry.addLine("");
            telemetry.addData(">>> SELECTED", selectedAlliance);
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
                .stopAndAdd(new AutoActions.FlywheelSetSpeed(robot, SHOOT_RPM))
                .waitSeconds(2.0)  //--- Give flywheel time to spin up
                //--- Align to shoot
                .strafeToSplineHeading(pos(-33.5, -36.5), degreeHeading(227))
                //--- Wait for flywheel to get up to speed, then fire all
                .stopAndAdd(new AutoActions.KickerWaitForSpeedThenFireAll(robot, "Preloads", fireLog))
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
                //--- Wait for flywheel to get up to speed, then fire all
                .stopAndAdd(new AutoActions.KickerWaitForSpeedThenFireAll(robot, "Stack 1", fireLog))
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
                //--- Wait for flywheel to get up to speed, then fire all
                .stopAndAdd(new AutoActions.KickerWaitForSpeedThenFireAll(robot, "Stack 2", fireLog))
                .build()
        );

        //========================================================================
        //--- Stack 3 --- PARK next to lever
        //========================================================================
        Actions.runBlocking(
            drive.actionBuilder(pose(-36, -36, 235))
                //--- Align with balls
                .strafeToSplineHeading(pos(14, -50), degreeHeading(10))
                //--- Turn on intake
                .stopAndAdd(new AutoActions.IntakeOn(robot))
                //--- Drive forward to pick up balls at half speed
                .strafeToSplineHeading(pos(30, -50), degreeHeading(0), slow(), slowAccel())
                //--- Reverse intake while moving to shoot position
                //.stopAndAdd(new AutoActions.IntakeReverse(robot))
                //--- Align to lever position
                .strafeToSplineHeading(pos(0, -50), degreeHeading(180))
                //--- Stop intake
                .stopAndAdd(new AutoActions.IntakeStop(robot))
                .build()
        );

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
                .stopAndAdd(new AutoActions.FlywheelSetSpeed(robot, SHOOT_RPM))
                .waitSeconds(2.0)
                //--- Blue: (-33.5, -36.5, 227°) → Red: (+33.5, -36.5, 313°)
                .strafeToSplineHeading(pos(33.5, -36.5), degreeHeading(313))
                .stopAndAdd(new AutoActions.KickerWaitForSpeedThenFireAll(robot, "Preloads", fireLog))
                .build()
        );

        //========================================================================
        //--- Stack 1, shoot (4-6)
        //========================================================================
        Actions.runBlocking(
            drive.actionBuilder(pose(36, -36, 305))
                //--- Blue: (-32, -50, 10°) → Red: (+32, -50, 170°)
                .strafeToSplineHeading(pos(32, -50), degreeHeading(170))
                .stopAndAdd(new AutoActions.IntakeOn(robot))
                //--- Blue: (-16, -50, 0°) → Red: (+16, -50, 180°)
                .strafeToSplineHeading(pos(16, -50), degreeHeading(180), slow(), slowAccel())
                //--- Blue: (-36, -36, 220°) → Red: (+36, -36, 320°)
                .strafeToSplineHeading(pos(36, -36), degreeHeading(320))
                .stopAndAdd(new AutoActions.IntakeStop(robot))
                .stopAndAdd(new AutoActions.KickerWaitForSpeedThenFireAll(robot, "Stack 1", fireLog))
                .build()
        );

        //========================================================================
        //--- Stack 2, shoot (7-9)
        //========================================================================
        Actions.runBlocking(
            drive.actionBuilder(pose(36, -36, 305))
                //--- Blue: (-9, -50, 10°) → Red: (+9, -50, 170°)
                .strafeToSplineHeading(pos(9, -50), degreeHeading(170))
                .stopAndAdd(new AutoActions.IntakeOn(robot))
                //--- Blue: (7, -50, 0°) → Red: (-7, -50, 180°)
                .strafeToSplineHeading(pos(-7, -50), degreeHeading(180), slow(), slowAccel())
                //--- Blue: (-36, -36, 224°) → Red: (+36, -36, 316°)
                .strafeToSplineHeading(pos(36, -36), degreeHeading(316))
                .stopAndAdd(new AutoActions.IntakeStop(robot))
                .stopAndAdd(new AutoActions.KickerWaitForSpeedThenFireAll(robot, "Stack 2", fireLog))
                .build()
        );

        //========================================================================
        //--- Stack 3 --- PARK next to lever
        //========================================================================
        Actions.runBlocking(
            drive.actionBuilder(pose(36, -36, 305))
                //--- Blue: (14, -50, 10°) → Red: (-14, -50, 170°)
                .strafeToSplineHeading(pos(-14, -50), degreeHeading(170))
                .stopAndAdd(new AutoActions.IntakeOn(robot))
                //--- Blue: (30, -50, 0°) → Red: (-30, -50, 180°)
                .strafeToSplineHeading(pos(-30, -50), degreeHeading(180), slow(), slowAccel())
                //--- Blue: (0, -50, 180°) → Red: (0, -50, 0°)
                .strafeToSplineHeading(pos(0, -50), degreeHeading(0))
                .stopAndAdd(new AutoActions.IntakeStop(robot))
                .build()
        );

        //--- Stop flywheel and intake
        Actions.runBlocking(new AutoActions.FlywheelStop(robot));
        Actions.runBlocking(new AutoActions.IntakeStop(robot));
    }
}
