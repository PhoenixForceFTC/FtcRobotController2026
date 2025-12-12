package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import java.util.Arrays;

public class AutoUtils {

   //--- Kinematics for wheel velocity constraint (same as MecanumDrive uses)
   private static final MecanumKinematics kinematics = new MecanumKinematics(
           MecanumDrive.PARAMS.inPerTick * MecanumDrive.PARAMS.trackWidthTicks, 
           MecanumDrive.PARAMS.inPerTick / MecanumDrive.PARAMS.lateralInPerTick);

   //==========================================================================
   //--- Static helper methods for cleaner trajectory code
   //==========================================================================

   /**
    * Creates a Vector2d position. Use with strafeToSplineHeading() etc.
    * Example: .strafeToSplineHeading(pos(-12, -36), heading(-135))
    *
    * @param x X-coordinate in inches
    * @param y Y-coordinate in inches
    * @return Vector2d position
    */
   public static Vector2d pos(double x, double y) {
       return new Vector2d(x, y);
   }

   /**
    * Converts degrees to radians for headings.
    * Example: .strafeToSplineHeading(pos(-12, -36), degreeHeading(-135))
    *
    * @param degrees Angle in degrees
    * @return Angle in radians
    */
   public static double degreeHeading(double degrees) {
       return Math.toRadians(degrees);
   }

   /**
    * Creates a Pose2d with position and heading (in degrees).
    * Example: Pose2d startPose = pose(-36, -60, 90);
    *
    * @param x       X-coordinate in inches
    * @param y       Y-coordinate in inches
    * @param degrees Heading in degrees
    * @return Pose2d with the specified position and heading
    */
   public static Pose2d pose(double x, double y, double degrees) {
       return new Pose2d(x, y, Math.toRadians(degrees));
   }

   /**
    * Creates a velocity constraint scaled by the given speed multiplier.
    * Use with trajectory methods that accept constraints.
    * Example: .strafeToSplineHeading(pos(-12, -36), heading(-135), slow())
    *
    * @param speedMultiplier The speed multiplier (1.0 = full, 0.5 = half speed, etc.)
    * @return VelConstraint scaled by the multiplier
    */
   public static VelConstraint speed(double speedMultiplier) {
       return new MinVelConstraint(Arrays.asList(
               kinematics.new WheelVelConstraint(MecanumDrive.PARAMS.maxWheelVel * speedMultiplier),
               new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel * speedMultiplier)
       ));
   }

   /**
    * Creates an acceleration constraint scaled by the given speed multiplier.
    * Use with trajectory methods that accept constraints.
    * Example: .strafeToSplineHeading(pos(-12, -36), heading(-135), slow(), slowAccel())
    *
    * @param speedMultiplier The accel multiplier (1.0 = full, 0.5 = half accel, etc.)
    * @return AccelConstraint scaled by the multiplier
    */
   public static AccelConstraint accel(double speedMultiplier) {
       return new ProfileAccelConstraint(
               MecanumDrive.PARAMS.minProfileAccel * speedMultiplier,
               MecanumDrive.PARAMS.maxProfileAccel * speedMultiplier
       );
   }

   //--- Convenience speed presets
   public static VelConstraint verySlow() { return speed(0.1); }
   public static VelConstraint slow() { return speed(0.5); }
   public static VelConstraint medium() { return speed(0.7); }
   public static VelConstraint fast() { return speed(1.0); }

   public static AccelConstraint verySlowAccel() { return accel(0.25); }
   public static AccelConstraint slowAccel() { return accel(0.5); }
   public static AccelConstraint mediumAccel() { return accel(0.7); }
   public static AccelConstraint fastAccel() { return accel(1.0); }
}