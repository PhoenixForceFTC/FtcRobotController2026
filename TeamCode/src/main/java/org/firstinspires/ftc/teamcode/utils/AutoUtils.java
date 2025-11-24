//package org.firstinspires.ftc.teamcode.utils;
//
//import com.acmerobotics.roadrunner.AccelConstraint;
//import com.acmerobotics.roadrunner.AngularVelConstraint;
//import com.acmerobotics.roadrunner.MinVelConstraint;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.ProfileAccelConstraint;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.VelConstraint;
//import com.acmerobotics.roadrunner.Action;
//import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
//import java.util.Arrays;
//
//public class AutoUtils {
//    private MecanumDrive drive;
//
//    /**
//     * Constructs an AutoUtils instance with the given MecanumDrive.
//     *
//     * @param drive The MecanumDrive instance used to build trajectories.
//     */
//    public AutoUtils(MecanumDrive drive) {
//        this.drive = drive;
//    }
//
//    /**
//     * Builds an Action to move the robot along a spline to the target (x, y)
//     * using the specified speed multiplier.
//     *
//     * @param startPose       The starting Pose2d for the trajectory.
//     * @param x               Target x-coordinate (in inches).
//     * @param y               Target y-coordinate (in inches).
//     * @param speedMultiplier The speed multiplier (1.0 = full, 0.7 = medium, 0.5 = slow).
//     * @return                An Action that executes the spline trajectory.
//     */
//    public Action moveSplineTo(Pose2d startPose, double x, double y, double speedMultiplier) {
//        //--- Create scaled velocity constraint using both wheel and angular velocity limits.
//        VelConstraint scaledVelConstraint = new MinVelConstraint(Arrays.asList(
//                drive.kinematics.new WheelVelConstraint(MecanumDrive.PARAMS.maxWheelVel * speedMultiplier),
//                new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel * speedMultiplier)
//        ));
//
//        //--- Create scaled acceleration constraint.
//        AccelConstraint scaledAccelConstraint = new ProfileAccelConstraint(
//                MecanumDrive.PARAMS.minProfileAccel * speedMultiplier,
//                MecanumDrive.PARAMS.maxProfileAccel * speedMultiplier
//        );
//
//        //--- Build the spline trajectory action using the scaled constraints.
//        Action action = drive.actionBuilder(startPose)
//                .splineTo(new Vector2d(x, y), 0, scaledVelConstraint, scaledAccelConstraint)
//                .build();
//
//        return action;
//    }
//}