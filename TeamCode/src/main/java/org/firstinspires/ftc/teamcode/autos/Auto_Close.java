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
    private static final double SHOOT_RPM = 2600.0;
    
    //--- Log of firing speeds for each phase
    private List<String> fireLog = new ArrayList<>();

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

        //========================================================================
        //--- PHASE 1: Shoot preloaded balls (1-3)
        //========================================================================
        Actions.runBlocking(
            drive.actionBuilder(startPose)
                //--- Start flywheel while driving
                .stopAndAdd(new AutoActions.FlywheelSetSpeed(robot, SHOOT_RPM))
                .waitSeconds(2.0)  //--- Give flywheel time to spool up
                //--- Align to shoot
                .strafeToSplineHeading(pos(-33.5, -36.5), degreeHeading(227))
                //--- Wait for flywheel to get up to speed, then fire all
                .stopAndAdd(new AutoActions.KickerWaitForSpeedThenFireAll(robot, "Phase 1 (Preload)", fireLog))
                .build()
        );

        //========================================================================
        //--- PHASE 2: Pick up balls from stack 1, shoot (4-6)
        //========================================================================
        Actions.runBlocking(
            drive.actionBuilder(pose(-36, -36, 235))
                //--- Align with balls
                .strafeToSplineHeading(pos(-32, -50), degreeHeading(10))
                //--- Turn on intake
                .stopAndAdd(new AutoActions.IntakeOn(robot))
                //--- Drive forward to pick up balls (stack 1) at half speed
                .strafeToSplineHeading(pos(-16, -50), degreeHeading(0), slow(), slowAccel())
                //.waitSeconds(1)
                //--- Reverse intake while moving to shoot position
                .stopAndAdd(new AutoActions.IntakeReverse(robot))
                //--- Align to shoot
                //.strafeToSplineHeading(pos(-33.5, -36.5), degreeHeading(227))
                .strafeToSplineHeading(pos(-36, -36), degreeHeading(220))
                //--- Stop intake
                .stopAndAdd(new AutoActions.IntakeStop(robot))
                //--- Wait for flywheel to get up to speed, then fire all
                .stopAndAdd(new AutoActions.KickerWaitForSpeedThenFireAll(robot, "Phase 2 (Stack 1)", fireLog))
                .build()
        );

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

        //--- Final telemetry with firing log
        telemetry.addData("Status", "Autonomous Complete!");
        telemetry.addLine("=== FIRING LOG ===");
        for (String entry : fireLog) {
            telemetry.addLine(entry);
        }
        telemetry.update();
        
        //--- Keep running while op mode is active
        while (opModeIsActive()) {
            sleep(100);
        }
    }
}
