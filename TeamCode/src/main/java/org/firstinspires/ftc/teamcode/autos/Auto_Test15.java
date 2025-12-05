package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.utils.AutoUtils.accel;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.degreeHeading;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.fast;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.fastAccel;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.medium;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.mediumAccel;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.pos;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.pose;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.slow;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.slowAccel;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.speed;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * Auto_TestPattern
 * 
 * Test autonomous that drives in a square pattern.
 * Each side uses a different speed/acceleration setting to demonstrate the helpers.
 * 
 * Pattern (24" square):
 *   Side 1: Forward  - SLOW (0.5)
 *   Side 2: Right    - MEDIUM (0.7)
 *   Side 3: Backward - FAST (1.0)
 *   Side 4: Left     - Custom (0.3)
 */
@Disabled
@Autonomous(name = "Auto: Test 15", group = "Test")
public class Auto_Test15 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        
        //--- Starting position (center of field for testing)
        Pose2d startPose = pose(0, 0, 90);
        
        //--- Initialize Road Runner drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        //--- Telemetry during init
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Pattern", "24\" Square with varying speeds");
        telemetry.addLine();
        telemetry.addData("Side 1", "Forward  - SLOW (0.5)");
        telemetry.addData("Side 2", "Right    - MEDIUM (0.7)");
        telemetry.addData("Side 3", "Backward - FAST (1.0)");
        telemetry.addData("Side 4", "Left     - Custom (0.3)");
        telemetry.update();

        //--- Wait for start
        waitForStart();

        if (isStopRequested()) return;

        //--- Drive in a 24" square pattern
        Actions.runBlocking(
            drive.actionBuilder(startPose)
                
                //--- Side 1: Forward 24" - SLOW
                .strafeToSplineHeading(pos(0, 24), degreeHeading(90), slow(), slowAccel())
                .waitSeconds(0.5)
                
                //--- Side 2: Right 24" - MEDIUM
                .strafeToSplineHeading(pos(24, 24), degreeHeading(90), medium(), mediumAccel())
                .waitSeconds(0.5)
                
                //--- Side 3: Backward 24" - FAST
                .strafeToSplineHeading(pos(24, 0), degreeHeading(90), fast(), fastAccel())
                .waitSeconds(0.5)
                
                //--- Side 4: Left 24" - Custom speed (0.3)
                .strafeToSplineHeading(pos(0, 0), degreeHeading(90), speed(0.3), accel(0.3))
                
                .build()
        );

        /*
        Psuedocodish (start angle 231.01000):
        Start motor
        start shooter
        Go back
        kick

        Go to 1st spike
        go back
        kick

        go to 2nd spike
        go back
        kick

        go to 3rd spike
        go back
        kick

        go to loading zone
        go back
        kick

        goto loading zone & shoot.

        */

        //--- Final telemetry
        telemetry.addData("Status", "Test Pattern Complete");
        telemetry.update();
    }
}
