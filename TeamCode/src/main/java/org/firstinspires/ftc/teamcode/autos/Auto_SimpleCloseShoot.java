package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;

import static org.firstinspires.ftc.teamcode.utils.AutoUtils.pos;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.degreeHeading;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.pose;

/**
 * Auto_SimpleCloseShoot
 * 
 * A simple autonomous that:
 * 1. Starts near the goal
 * 2. Spins up the flywheel and intake
 * 3. Moves to a shooting position
 * 4. Waits for flywheel to reach speed
 * 5. Fires all three balls
 * 
 * Starting Position: Close to goal, facing the goal
 */
@Disabled
@Autonomous(name = "Auto: Simple Close Shoot", group = "Auto")
public class Auto_SimpleCloseShoot extends LinearOpMode {

    //--- Robot hardware
    private RobotHardware robot;
    
    //--- Flywheel speed for close shot
    private static final double SHOOT_RPM = 2500.0;

    @Override
    public void runOpMode() throws InterruptedException {
        
        //--- Initialize robot hardware
        robot = new RobotHardware(this);
        robot.init(1);
        
        //--- Starting position (adjust based on actual field placement)
        //--- X: negative is towards red alliance wall, positive is towards blue
        //--- Y: negative is towards audience, positive is towards back wall
        //--- Heading: 0 is facing positive X, 90 is facing positive Y
        Pose2d startPose = pose(-36, -60, 90);  // Near goal, facing forward
        
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

        //--- Build and run the autonomous trajectory
        Actions.runBlocking(
            drive.actionBuilder(startPose)
                
                //--- Step 1: Start flywheel and intake immediately
                .stopAndAdd(new AutoActions.PrepareToShoot(robot, SHOOT_RPM))
                
                //--- Step 2: Move to shooting position (strafe to align with goal)
                .strafeToSplineHeading(pos(-12, -36), degreeHeading(-135))  // Move and turn to face goal
                
                //--- Step 3: Wait for flywheel to spin up
                .stopAndAdd(new AutoActions.FlywheelWaitForSpeed(robot))
                
                //--- Step 4: Small pause to stabilize
                .waitSeconds(0.3)
                
                //--- Step 5: Fire all three kickers
                .stopAndAdd(new AutoActions.KickerFireAll(robot))
                
                //--- Step 6: Wait for kickers to complete
                .waitSeconds(0.5)
                
                //--- Step 7: Retract kickers
                .stopAndAdd(new AutoActions.KickerRetractAll(robot))
                
                //--- Step 8: Stop flywheel and intake
                .stopAndAdd(new AutoActions.FlywheelStop(robot))
                .stopAndAdd(new AutoActions.IntakeStop(robot))
                
                //--- Step 9: Park (optional - move out of the way)
                .strafeToSplineHeading(pos(-36, -36), degreeHeading(90))
                
                .build()
        );

        //--- Final telemetry
        telemetry.addData("Status", "Autonomous Complete");
        telemetry.update();
    }
}
