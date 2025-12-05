package org.firstinspires.ftc.teamcode.autos;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * AutoActions - Road Runner Action classes for autonomous routines
 * 
 * These actions can be used with .stopAndAdd() in trajectory builders
 * to perform robot actions during autonomous.
 */
public class AutoActions {

    //region --- Kicker Actions ---

    /**
     * Fire a single kicker (1=Left, 2=Middle, 3=Right)
     */
    public static class KickerFire implements Action {
        private final RobotHardware robot;
        private final int kickerPos;

        public KickerFire(RobotHardware robot, int kickerPos) {
            this.robot = robot;
            this.kickerPos = kickerPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.kickers.fireKicker(kickerPos);
            return false;  // Action completes immediately (servo continues in background)
        }
    }

    /**
     * Fire all three kickers up (does not wait or retract)
     */
    public static class KickerFireUp implements Action {
        private final RobotHardware robot;

        public KickerFireUp(RobotHardware robot) {
            this.robot = robot;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.kickers.fireAll();
            return false;
        }
    }

    /**
     * Fire all kickers, wait 0.5s, then retract - complete firing sequence
     */
    public static class KickerFireAll implements Action {
        private final RobotHardware robot;
        private ElapsedTime timer;
        private int state = 0;
        private static final double FIRE_HOLD_TIME = 0.5;

        public KickerFireAll(RobotHardware robot) {
            this.robot = robot;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                robot.kickers.fireAll();
                state = 0;
            }

            switch (state) {
                case 0:  // Wait for kick to complete
                    if (timer.seconds() > FIRE_HOLD_TIME) {
                        robot.kickers.retractAll();
                        state = 1;
                        return false;  // Done
                    }
                    return true;

                default:
                    return false;
            }
        }
    }

    /**
     * Wait for flywheel to reach speed, settle briefly, then fire all kickers, wait, and retract
     * Complete shooting sequence in one action
     * Uses wider tolerance than hardware class for faster firing with acceptable accuracy
     */
    public static class KickerWaitForSpeedThenFireAll implements Action {
        private final RobotHardware robot;
        private final String label;
        private final java.util.List<String> fireLog;
        private ElapsedTime timer;
        private ElapsedTime stableTimer;
        private int state = 0;
        private double timeFirstStable = -1;  // Track when we first achieved stable speed
        private static final double FLYWHEEL_TIMEOUT = 6.0;
        private static final double STABLE_TIME = 0.1;  // Must stay at target continuously for this long
        private static final double FIRE_HOLD_TIME = 0.5;
        private static final double RPM_TOLERANCE = 75.0;  // Wider tolerance for faster firing

        public KickerWaitForSpeedThenFireAll(RobotHardware robot) {
            this(robot, null, null);
        }

        public KickerWaitForSpeedThenFireAll(RobotHardware robot, String label, java.util.List<String> fireLog) {
            this.robot = robot;
            this.label = label;
            this.fireLog = fireLog;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                stableTimer = new ElapsedTime();
            }

            switch (state) {
                case 0:  // Wait for flywheel to be continuously stable
                    robot.flywheel.run();  // Update flywheel state
                    double currentRPM = robot.flywheel.getCurrentRPM();
                    double targetRPM = robot.flywheel.getTargetRPM();
                    boolean atTarget = Math.abs(currentRPM - targetRPM) <= RPM_TOLERANCE;
                    
                    // Reset stable timer if we leave target range
                    if (!atTarget) {
                        stableTimer.reset();
                        timeFirstStable = -1;
                    } else if (timeFirstStable < 0) {
                        // Just entered target range
                        timeFirstStable = timer.seconds();
                    }
                    
                    // Fire if we've been stable continuously, or overall timeout
                    boolean stable = atTarget && stableTimer.seconds() > STABLE_TIME;
                    boolean timedOut = timer.seconds() > FLYWHEEL_TIMEOUT;
                    
                    if (stable || timedOut) {
                        // Log the firing speed with timing info
                        double firingRPM = robot.flywheel.getCurrentRPM();
                        double totalTime = timer.seconds();
                        if (fireLog != null && label != null) {
                            String reason = timedOut && !stable ? " (TIMEOUT!)" : " (stable)";
                            String hitInfo = timeFirstStable >= 0 
                                ? String.format(" [stable@%.2fs, fire@%.2fs]", timeFirstStable, totalTime)
                                : String.format(" [never stable, fire@%.2fs]", totalTime);
                            fireLog.add(String.format("%s: %.0f RPM%s%s", label, firingRPM, reason, hitInfo));
                        }
                        
                        robot.kickers.fireAll();
                        timer.reset();
                        state = 1;
                    }
                    telemetryPacket.put("Flywheel Target", robot.flywheel.getTargetRPM());
                    telemetryPacket.put("Flywheel Current", robot.flywheel.getCurrentRPM());
                    telemetryPacket.put("At Target", atTarget);
                    telemetryPacket.put("Stable Time", stableTimer.seconds());
                    return true;

                case 1:  // Wait for kick to complete
                    if (timer.seconds() > FIRE_HOLD_TIME) {
                        robot.kickers.retractAll();
                        state = 2;
                        return false;  // Done
                    }
                    return true;

                default:
                    return false;
            }
        }
    }

    /**
     * Fire kickers in sequence based on ball colors and target sequence
     * Note: This starts the sequence but doesn't wait for completion
     */
    public static class KickerFireSequence implements Action {
        private final RobotHardware robot;

        public KickerFireSequence(RobotHardware robot) {
            this.robot = robot;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.kickers.fireSequence();
            return false;
        }
    }

    /**
     * Retract all kickers to down position
     */
    public static class KickerRetractAll implements Action {
        private final RobotHardware robot;

        public KickerRetractAll(RobotHardware robot) {
            this.robot = robot;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.kickers.retractAll();
            return false;
        }
    }

    //endregion

    //region --- Flywheel Actions ---

    /**
     * Set flywheel to a specific RPM
     */
    public static class FlywheelSetSpeed implements Action {
        private final RobotHardware robot;
        private final double rpm;

        public FlywheelSetSpeed(RobotHardware robot, double rpm) {
            this.robot = robot;
            this.rpm = rpm;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.flywheel.setVelocity(rpm);
            return false;
        }
    }

    /**
     * Wait for flywheel to reach target velocity
     * This action blocks until velocity is reached
     */
    public static class FlywheelWaitForSpeed implements Action {
        private final RobotHardware robot;
        private ElapsedTime timer;
        private static final double TIMEOUT = 3.0;  // Max wait time in seconds

        public FlywheelWaitForSpeed(RobotHardware robot) {
            this.robot = robot;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
            }

            // Update flywheel state and check if at target or timed out
            robot.flywheel.run();
            if (robot.flywheel.isAtTarget() || timer.seconds() > TIMEOUT) {
                return false;  // Done waiting
            }

            telemetryPacket.put("Flywheel Target", robot.flywheel.getTargetRPM());
            telemetryPacket.put("Flywheel Current", robot.flywheel.getCurrentRPM());
            return true;  // Keep waiting
        }
    }

    /**
     * Stop the flywheel
     */
    public static class FlywheelStop implements Action {
        private final RobotHardware robot;

        public FlywheelStop(RobotHardware robot) {
            this.robot = robot;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.flywheel.stop();
            return false;
        }
    }

    //endregion

    //region --- Intake Actions ---

    /**
     * Start intake (pulling balls in)
     */
    public static class IntakeOn implements Action {
        private final RobotHardware robot;

        public IntakeOn(RobotHardware robot) {
            this.robot = robot;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.intake.intake();
            return false;
        }
    }

    /**
     * Start outtake (pushing balls out - reverse)
     */
    public static class IntakeReverse implements Action {
        private final RobotHardware robot;

        public IntakeReverse(RobotHardware robot) {
            this.robot = robot;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.intake.outtake();
            return false;
        }
    }

    /**
     * Stop the intake
     */
    public static class IntakeStop implements Action {
        private final RobotHardware robot;

        public IntakeStop(RobotHardware robot) {
            this.robot = robot;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.intake.stop();
            return false;
        }
    }

    //endregion

    //region --- Compound Actions ---

    /**
     * Prepare to shoot - starts flywheel and intake
     */
    public static class PrepareToShoot implements Action {
        private final RobotHardware robot;
        private final double rpm;

        public PrepareToShoot(RobotHardware robot, double rpm) {
            this.robot = robot;
            this.rpm = rpm;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.flywheel.setVelocity(rpm);
            robot.intake.intake();
            return false;
        }
    }

    /**
     * Full shoot sequence - waits for flywheel, fires all, retracts
     * This is a blocking action that handles the full sequence
     */
    public static class ShootAll implements Action {
        private final RobotHardware robot;
        private ElapsedTime timer;
        private int state = 0;
        private static final double FLYWHEEL_TIMEOUT = 3.0;
        private static final double FIRE_HOLD_TIME = 0.5;

        public ShootAll(RobotHardware robot) {
            this.robot = robot;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
            }

            switch (state) {
                case 0:  // Wait for flywheel
                    if (robot.flywheel.isAtTarget() || timer.seconds() > FLYWHEEL_TIMEOUT) {
                        robot.kickers.fireAll();
                        timer.reset();
                        state = 1;
                    }
                    return true;

                case 1:  // Wait for kick to complete
                    if (timer.seconds() > FIRE_HOLD_TIME) {
                        robot.kickers.retractAll();
                        state = 2;
                        return false;  // Done
                    }
                    return true;

                default:
                    return false;
            }
        }
    }

    //endregion

    //region --- Utility Actions ---

    /**
     * Run the robot hardware update loop
     * Call this periodically during long waits to keep hardware responsive
     */
    public static class RobotRun implements Action {
        private final RobotHardware robot;

        public RobotRun(RobotHardware robot) {
            this.robot = robot;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.run();
            return false;
        }
    }

    //endregion
}
