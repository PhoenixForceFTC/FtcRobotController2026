package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;

import static org.firstinspires.ftc.teamcode.utils.AutoUtils.pos;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.degreeHeading;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.pose;

/**
 * Auto_Close
 * 
 * Full autonomous routine that:
 * 1. Starts at close position (-50, -50) facing 235°
 * 2. Shoots preloaded balls (1-3)
 * 3. Picks up balls from first stack, shoots (4-6)
 * 4. Opens second stack, picks up, shoots (7-9)
 * 5. Picks up third stack, shoots (10-12)
 * 6. Picks up from player zone, shoots (13-15)
 * 7. Parks off the line
 * 
 * Starting Position: Close to goal corner
 */
@Autonomous(name = "Auto: Close", group = "Auto")
public class Auto_Close extends LinearOpMode {

    //--- Robot hardware
    private RobotHardware robot;
    
    //--- Flywheel speed for shooting
    private static final double SHOOT_RPM = 2400.0;

    @Override
    public void runOpMode() throws InterruptedException {
        
        //--- Initialize robot hardware
        robot = new RobotHardware(this);
        robot.init(2);  // Beta robot
        
        //--- Starting position
        Pose2d startPose = pose(-55, -45, 235);
        
        //--- Initialize Road Runner drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        //--- Telemetry during init
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start Pose", "X: %.1f, Y: %.1f, Heading: %.1f°", 
                startPose.position.x, startPose.position.y, Math.toDegrees(startPose.heading.toDouble()));
        telemetry.update();

        //--- Wait for start
        waitForStart();

        if (isStopRequested()) return;

        //--- Start flywheel and intake for the whole auto
        Actions.runBlocking(new AutoActions.PrepareToShoot(robot, SHOOT_RPM));

        //========================================================================
        //--- PHASE 1: Shoot preloaded balls (1-3)
        //========================================================================
        Actions.runBlocking(
            drive.actionBuilder(startPose)
                //--- Align to shoot
                .strafeToSplineHeading(pos(-36, -36), degreeHeading(235))
                //--- Wait for flywheel
                .stopAndAdd(new AutoActions.FlywheelWaitForSpeed(robot))
                .waitSeconds(0.3)
                //--- Fire all
                .stopAndAdd(new AutoActions.KickerFireAll(robot))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerRetractAll(robot))
                .build()
        );

        //========================================================================
        //--- PHASE 2: Pick up balls from stack 1, shoot (4-6)
        //========================================================================
        // Actions.runBlocking(
        //     drive.actionBuilder(pose(-36, -36, 235))
        //         //--- Align with balls
        //         .strafeToSplineHeading(pos(-26, -46), degreeHeading(0))
        //         //--- Drive forward to pick up balls (stack 1)
        //         .strafeToSplineHeading(pos(-16, -46), degreeHeading(0))
        //         //--- Align to shoot
        //         .strafeToSplineHeading(pos(-26, -46), degreeHeading(0))
        //         .strafeToSplineHeading(pos(-36, -36), degreeHeading(235))
        //         //--- Wait for flywheel
        //         .stopAndAdd(new AutoActions.FlywheelWaitForSpeed(robot))
        //         .waitSeconds(0.3)
        //         //--- Fire all
        //         .stopAndAdd(new AutoActions.KickerFireAll(robot))
        //         .waitSeconds(0.5)
        //         .stopAndAdd(new AutoActions.KickerRetractAll(robot))
        //         .build()
        // );

        //========================================================================
        //--- PHASE 3: Open stack 2, pick up, shoot (7-9)
        //========================================================================
        // Actions.runBlocking(
        //     drive.actionBuilder(pose(-36, -36, 235))
        //         //--- Align with balls
        //         .strafeToSplineHeading(pos(-26, -46), degreeHeading(0))
        //         //--- Open the balls (push into stack)
        //         .strafeToSplineHeading(pos(-5, -46), degreeHeading(0))
        //         .strafeToSplineHeading(pos(-5, -50), degreeHeading(0))
        //         .strafeToSplineHeading(pos(-5, -46), degreeHeading(0))
        //         //--- Drive forward to pick up balls (stack 2)
        //         .strafeToSplineHeading(pos(7, -46), degreeHeading(0))
        //         //--- Align to shoot
        //         .strafeToSplineHeading(pos(-26, -46), degreeHeading(0))
        //         .strafeToSplineHeading(pos(-36, -36), degreeHeading(235))
        //         //--- Wait for flywheel
        //         .stopAndAdd(new AutoActions.FlywheelWaitForSpeed(robot))
        //         .waitSeconds(0.3)
        //         //--- Fire all
        //         .stopAndAdd(new AutoActions.KickerFireAll(robot))
        //         .waitSeconds(0.5)
        //         .stopAndAdd(new AutoActions.KickerRetractAll(robot))
        //         .build()
        // );

        //========================================================================
        //--- PHASE 4: Pick up stack 3, shoot (10-12)
        //========================================================================
        // Actions.runBlocking(
        //     drive.actionBuilder(pose(-36, -36, 235))
        //         //--- Align with balls
        //         .strafeToSplineHeading(pos(-26, -46), degreeHeading(0))
        //         //--- Drive forward to pick up balls (stack 3)
        //         .strafeToSplineHeading(pos(30, -46), degreeHeading(0))
        //         //--- Align to shoot
        //         .strafeToSplineHeading(pos(-26, -46), degreeHeading(0))
        //         .strafeToSplineHeading(pos(-36, -36), degreeHeading(235))
        //         //--- Wait for flywheel
        //         .stopAndAdd(new AutoActions.FlywheelWaitForSpeed(robot))
        //         .waitSeconds(0.3)
        //         //--- Fire all
        //         .stopAndAdd(new AutoActions.KickerFireAll(robot))
        //         .waitSeconds(0.5)
        //         .stopAndAdd(new AutoActions.KickerRetractAll(robot))
        //         .build()
        // );

        //========================================================================
        //--- PHASE 5: Pick up from player zone, shoot (13-15)
        //========================================================================
        // Actions.runBlocking(
        //     drive.actionBuilder(pose(-36, -36, 235))
        //         //--- Align with balls
        //         .strafeToSplineHeading(pos(-26, -46), degreeHeading(0))
        //         //--- Drive to player zone (stack 4)
        //         .strafeToSplineHeading(pos(50, -46), degreeHeading(0))
        //         .strafeToSplineHeading(pos(60, -50), degreeHeading(270))
        //         .strafeToSplineHeading(pos(60, -60), degreeHeading(270))
        //         //--- Back out of player zone
        //         .strafeToSplineHeading(pos(50, -46), degreeHeading(0))
        //         //--- Align to shoot
        //         .strafeToSplineHeading(pos(-26, -46), degreeHeading(0))
        //         .strafeToSplineHeading(pos(-36, -36), degreeHeading(235))
        //         //--- Wait for flywheel
        //         .stopAndAdd(new AutoActions.FlywheelWaitForSpeed(robot))
        //         .waitSeconds(0.3)
        //         //--- Fire all
        //         .stopAndAdd(new AutoActions.KickerFireAll(robot))
        //         .waitSeconds(0.5)
        //         .stopAndAdd(new AutoActions.KickerRetractAll(robot))
        //         .build()
        // );

        //========================================================================
        //--- PHASE 6: Park off the line
        //========================================================================
        // Actions.runBlocking(
        //     drive.actionBuilder(pose(-36, -36, 235))
        //         //--- Move off the line
        //         .strafeToSplineHeading(pos(-36, -50), degreeHeading(235))
        //         .build()
        // );

        //--- Stop flywheel and intake
        Actions.runBlocking(new AutoActions.FlywheelStop(robot));
        Actions.runBlocking(new AutoActions.IntakeStop(robot));

        //--- Final telemetry
        telemetry.addData("Status", "Autonomous Complete - 15 balls fired!");
        telemetry.update();
        
        //--- Keep running while op mode is active
        while (opModeIsActive()) {
            sleep(100);
        }
    }
}
