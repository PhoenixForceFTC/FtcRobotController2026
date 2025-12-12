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
     * Automatically adjusts flywheel RPM based on ball count in intake
     * 
     * @param robot The robot hardware
     * @param rpm1Ball RPM for 1 ball (depends on shooting position)
     * @param rpm2Balls RPM for 2 balls (depends on shooting position)
     * @param rpm3Balls RPM for 3 balls (depends on shooting position)
     */
    public static class KickerWaitForSpeedThenFireAll implements Action {
        private final RobotHardware robot;
        private final String label;
        private final java.util.List<String> fireLog;
        private final double rpm1Ball;
        private final double rpm2Balls;
        private final double rpm3Balls;
        private ElapsedTime timer;
        private ElapsedTime stableTimer;
        private int state = 0;
        private double timeFirstStable = -1;  // Track when we first achieved stable speed
        private boolean rpmSet = false;       // Track if we've set RPM based on ball count
        private int ballCount = 0;            // Number of balls detected in intake
        private static final double FLYWHEEL_TIMEOUT = 6.0;
        private static final double STABLE_TIME = 0.1;  // Must stay at target continuously for this long
        private static final double FIRE_HOLD_TIME = 0.5;
        private static final double RPM_TOLERANCE = 75.0;  // Wider tolerance for faster firing

        public KickerWaitForSpeedThenFireAll(RobotHardware robot, double rpm1Ball, double rpm2Balls, double rpm3Balls) {
            this(robot, rpm1Ball, rpm2Balls, rpm3Balls, null, null);
        }

        public KickerWaitForSpeedThenFireAll(RobotHardware robot, double rpm1Ball, double rpm2Balls, double rpm3Balls, 
                                              String label, java.util.List<String> fireLog) {
            this.robot = robot;
            this.rpm1Ball = rpm1Ball;
            this.rpm2Balls = rpm2Balls;
            this.rpm3Balls = rpm3Balls;
            this.label = label;
            this.fireLog = fireLog;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                stableTimer = new ElapsedTime();
            }
            
            //--- On first run, read ball count and set RPM accordingly
            if (!rpmSet) {
                ballCount = robot.intake.getBallCount();
                double rpm;
                if (ballCount <= 1) {
                    rpm = rpm1Ball;
                } else if (ballCount == 2) {
                    rpm = rpm2Balls;
                } else {
                    rpm = rpm3Balls;
                }
                robot.flywheel.setVelocity(rpm);
                rpmSet = true;
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
                        // Log the firing speed with timing info and ball count
                        double firingRPM = robot.flywheel.getCurrentRPM();
                        double totalTime = timer.seconds();
                        if (fireLog != null && label != null) {
                            String reason = timedOut && !stable ? " (TIMEOUT!)" : " (stable)";
                            String hitInfo = timeFirstStable >= 0 
                                ? String.format(" [stable@%.2fs, fire@%.2fs]", timeFirstStable, totalTime)
                                : String.format(" [never stable, fire@%.2fs]", totalTime);
                            fireLog.add(String.format("%s: %.0f RPM, %d balls%s%s", label, firingRPM, ballCount, reason, hitInfo));
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
     * Wait for flywheel to reach speed, then fire kickers in sequence based on 
     * detected ball colors and target sequence pattern.
     * 
     * This action:
     * 1. Sets flywheel to specified RPM (single-ball RPM for sequence firing)
     * 2. Waits for flywheel to reach target RPM
     * 3. Reads ball colors from intake sensors and configures kickers
     * 4. Sets the firing sequence from the detected BallSequence
     * 5. Fires each kicker in the correct order, waiting for velocity recovery between shots
     * 
     * The sequence determines which color ball to fire first:
     * - GPP: Fire Green first, then Purple, then Purple
     * - PGP: Fire Purple first, then Green, then Purple
     * - PPG: Fire Purple first, then Purple, then Green
     */
    public static class KickerWaitForSpeedThenFireSequence implements Action {
        private final RobotHardware robot;
        private final double targetRPM;
        private final String label;
        private final java.util.List<String> fireLog;
        private final ElapsedTime autoTimer;  // Global auto timer for timestamps
        private ElapsedTime timer;
        private ElapsedTime stableTimer;
        private int state = 0;
        private double timeFirstStable = -1;
        private boolean rpmSet = false;
        private int lastLoggedStep = -1;  // Track which step we last logged
        private static final double FLYWHEEL_TIMEOUT = 6.0;
        private static final double STABLE_TIME = 0.1;
        private static final double RPM_TOLERANCE = 75.0;

        public KickerWaitForSpeedThenFireSequence(RobotHardware robot, double targetRPM) {
            this(robot, targetRPM, null, null, null);
        }

        public KickerWaitForSpeedThenFireSequence(RobotHardware robot, double targetRPM, String label, java.util.List<String> fireLog) {
            this(robot, targetRPM, label, fireLog, null);
        }

        public KickerWaitForSpeedThenFireSequence(RobotHardware robot, double targetRPM, String label, java.util.List<String> fireLog, ElapsedTime autoTimer) {
            this.robot = robot;
            this.targetRPM = targetRPM;
            this.label = label;
            this.fireLog = fireLog;
            this.autoTimer = autoTimer;
        }

        /** Get timestamp string for log entries (e.g., " 12s") */
        private String getTimestamp() {
            if (autoTimer == null) return "";
            return String.format(" %.0fs", autoTimer.seconds());
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                stableTimer = new ElapsedTime();
            }
            
            // Set flywheel RPM on first run
            if (!rpmSet) {
                robot.flywheel.setVelocity(targetRPM);
                rpmSet = true;
            }

            switch (state) {
                case 0:  // Wait for flywheel to be continuously stable
                    robot.flywheel.run();
                    robot.intake.updateBallDetection();  // Force sensor update even when intake off
                    double currentRPM = robot.flywheel.getCurrentRPM();
                    double flywheelTarget = robot.flywheel.getTargetRPM();
                    boolean atTarget = Math.abs(currentRPM - flywheelTarget) <= RPM_TOLERANCE;
                    
                    if (!atTarget) {
                        stableTimer.reset();
                        timeFirstStable = -1;
                    } else if (timeFirstStable < 0) {
                        timeFirstStable = timer.seconds();
                    }
                    
                    boolean stable = atTarget && stableTimer.seconds() > STABLE_TIME;
                    boolean timedOut = timer.seconds() > FLYWHEEL_TIMEOUT;
                    
                    if (stable || timedOut) {
                        // Read ball colors from intake and configure kickers
                        configureBallColorsFromIntake();
                        
                        // Set the firing sequence from detected sequence
                        configureSequenceFromCamera();
                        
                        if (fireLog != null && label != null) {
                            String reason = timedOut && !stable ? "TIMEOUT" : "stable";
                            String seqInfo = String.format("Seq:%s", robot.kickers.getSequence());
                            fireLog.add(String.format("%s START: target=%.0f (%s) %s%s", label, targetRPM, reason, seqInfo, getTimestamp()));
                        }
                        
                        // Start the sequence firing
                        robot.kickers.fireSequence();
                        lastLoggedStep = -1;  // Reset for tracking individual fires
                        state = 1;
                    }
                    telemetryPacket.put("Flywheel Target", robot.flywheel.getTargetRPM());
                    telemetryPacket.put("Flywheel Current", robot.flywheel.getCurrentRPM());
                    telemetryPacket.put("At Target", atTarget);
                    telemetryPacket.put("Stable Time", stableTimer.seconds());
                    return true;

                case 1:  // Wait for sequence to complete
                    robot.flywheel.run();  // Keep flywheel running for velocity recovery
                    robot.intake.updateBallDetection();  // Keep color sensors updated
                    robot.kickers.run();   // Run kicker state machine
                    
                    // Log each individual ball fire as it happens
                    int currentStep = robot.kickers.getSequenceStep();
                    if (currentStep > lastLoggedStep && currentStep <= 3) {
                        // A new ball just fired (step increments after fire completes)
                        double actualRPM = robot.flywheel.getCurrentRPM();
                        double target = robot.flywheel.getTargetRPM();
                        int[] order = robot.kickers.getFiringOrder();
                        int kickerFired = (lastLoggedStep >= 0 && lastLoggedStep < 3) ? order[lastLoggedStep] : 0;
                        String kickerName = (kickerFired == 1) ? "L" : (kickerFired == 2) ? "M" : (kickerFired == 3) ? "R" : "?";
                        
                        // Get ball color at kicker position and expected color for this step
                        String detectedColor = "?";
                        String expectedColor = "?";
                        if (kickerFired >= 1 && kickerFired <= 3 && lastLoggedStep >= 0) {
                            org.firstinspires.ftc.teamcode.hardware.Kickers.BallColor detected = 
                                robot.kickers.getBallColor(kickerFired);
                            org.firstinspires.ftc.teamcode.hardware.Kickers.BallColor expected = 
                                robot.kickers.getExpectedColorForStep(lastLoggedStep);
                            detectedColor = (detected == org.firstinspires.ftc.teamcode.hardware.Kickers.BallColor.GREEN) ? "G" : "P";
                            expectedColor = (expected == org.firstinspires.ftc.teamcode.hardware.Kickers.BallColor.GREEN) ? "G" : "P";
                        }
                        
                        if (fireLog != null && label != null && lastLoggedStep >= 0) {
                            fireLog.add(String.format("%s #%d (%s): %.0f/%.0f RPM %s/%s%s", 
                                label, lastLoggedStep + 1, kickerName, actualRPM, target, detectedColor, expectedColor, getTimestamp()));
                        }
                        lastLoggedStep = currentStep;
                    }
                    
                    // Also log the first ball when sequence starts
                    if (lastLoggedStep == -1 && robot.kickers.isSequenceFiring()) {
                        lastLoggedStep = 0;
                    }
                    
                    if (robot.kickers.isSequenceComplete()) {
                        // Log the final (3rd) ball
                        if (fireLog != null && label != null && lastLoggedStep == 2) {
                            double actualRPM = robot.flywheel.getCurrentRPM();
                            double target = robot.flywheel.getTargetRPM();
                            int[] order = robot.kickers.getFiringOrder();
                            int kickerFired = order[2];
                            String kickerName = (kickerFired == 1) ? "L" : (kickerFired == 2) ? "M" : (kickerFired == 3) ? "R" : "?";
                            
                            // Get ball color at kicker position and expected color for step 2
                            org.firstinspires.ftc.teamcode.hardware.Kickers.BallColor detected = 
                                robot.kickers.getBallColor(kickerFired);
                            org.firstinspires.ftc.teamcode.hardware.Kickers.BallColor expected = 
                                robot.kickers.getExpectedColorForStep(2);
                            String detectedColor = (detected == org.firstinspires.ftc.teamcode.hardware.Kickers.BallColor.GREEN) ? "G" : "P";
                            String expectedColor = (expected == org.firstinspires.ftc.teamcode.hardware.Kickers.BallColor.GREEN) ? "G" : "P";
                            
                            fireLog.add(String.format("%s #3 (%s): %.0f/%.0f RPM %s/%s%s", 
                                label, kickerName, actualRPM, target, detectedColor, expectedColor, getTimestamp()));
                        }
                        state = 2;
                        return false;  // Done
                    }
                    telemetryPacket.put("Sequence", "Firing...");
                    telemetryPacket.put("Step", currentStep);
                    return true;

                default:
                    return false;
            }
        }
        
        /**
         * Read ball colors from intake sensors and configure kickers
         */
        private void configureBallColorsFromIntake() {
            org.firstinspires.ftc.teamcode.hardware.Intake.BallColor[] intakeColors = 
                robot.intake.getAllShooterBallColors();
            
            // Convert Intake.BallColor to Kickers.BallColor for each position
            for (int i = 0; i < 3; i++) {
                org.firstinspires.ftc.teamcode.hardware.Kickers.BallColor kickerColor = 
                    convertBallColor(intakeColors[i]);
                robot.kickers.setBallColor(i + 1, kickerColor);
            }
        }
        
        /**
         * Convert Intake.BallColor to Kickers.BallColor
         */
        private org.firstinspires.ftc.teamcode.hardware.Kickers.BallColor convertBallColor(
                org.firstinspires.ftc.teamcode.hardware.Intake.BallColor intakeColor) {
            switch (intakeColor) {
                case GREEN:
                    return org.firstinspires.ftc.teamcode.hardware.Kickers.BallColor.GREEN;
                case PURPLE:
                default:
                    return org.firstinspires.ftc.teamcode.hardware.Kickers.BallColor.PURPLE;
            }
        }
        
        /**
         * Set the kicker firing sequence based on detected camera sequence
         */
        private void configureSequenceFromCamera() {
            org.firstinspires.ftc.teamcode.hardware.Camera.BallSequence detected = 
                robot.camera.getDetectedSequence();
            
            org.firstinspires.ftc.teamcode.hardware.Kickers.Sequence kickerSequence;
            switch (detected) {
                case GPP:
                    kickerSequence = org.firstinspires.ftc.teamcode.hardware.Kickers.Sequence.GPP;
                    break;
                case PGP:
                    kickerSequence = org.firstinspires.ftc.teamcode.hardware.Kickers.Sequence.PGP;
                    break;
                case PPG:
                    kickerSequence = org.firstinspires.ftc.teamcode.hardware.Kickers.Sequence.PPG;
                    break;
                default:
                    // Default to GPP if unknown
                    kickerSequence = org.firstinspires.ftc.teamcode.hardware.Kickers.Sequence.GPP;
                    break;
            }
            robot.kickers.setSequence(kickerSequence);
        }
    }

    /**
     * Auto-aim using camera, wait for alignment AND flywheel speed, then fire in sequence.
     * 
     * This action:
     * 1. Enables camera auto-alignment (robot rotates to center target in camera view)
     * 2. Gets distance-based RPM from camera (uses 1-ball table for sequence firing)
     * 3. Waits for BOTH alignment AND flywheel to reach target
     * 4. Reads ball colors from intake sensors and configures kickers
     * 5. Sets the firing sequence from the detected BallSequence
     * 6. Fires each kicker in the correct order, waiting for velocity recovery between shots
     * 7. Disables auto-alignment when complete
     * 
     * Use this when the robot can see the target and should auto-correct its aim.
     */
    public static class AutoAimAndFireSequence implements Action {
        private final RobotHardware robot;
        private final String label;
        private final java.util.List<String> fireLog;
        private final double defaultRPM;
        private ElapsedTime timer;
        private ElapsedTime stableTimer;
        private int state = 0;
        private double timeFirstStable = -1;
        private double timeFirstAligned = -1;
        private double timeFirstTargetSeen = -1;
        private boolean initialized = false;
        private double targetRPM = 0;
        private double initialDistance = -1;
        private int lastDetectedTag = -1;
        private static final double TIMEOUT = 6.0;          // Max time for aiming + spinup
        private static final double STABLE_TIME = 0.1;       // Must be at target continuously
        private static final double RPM_TOLERANCE = 75.0;    // Wider tolerance for faster firing
        private static final double SCAN_WAIT_TIME = 1.5;    // Time to wait for camera to find target
        private static final double DEFAULT_FALLBACK_RPM = 2250.0;  // Used if no defaultRPM provided

        public AutoAimAndFireSequence(RobotHardware robot) {
            this(robot, DEFAULT_FALLBACK_RPM, null, null);
        }

        public AutoAimAndFireSequence(RobotHardware robot, String label, java.util.List<String> fireLog) {
            this(robot, DEFAULT_FALLBACK_RPM, label, fireLog);
        }

        public AutoAimAndFireSequence(RobotHardware robot, double defaultRPM, String label, java.util.List<String> fireLog) {
            this.robot = robot;
            this.defaultRPM = defaultRPM;
            this.label = label;
            this.fireLog = fireLog;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                stableTimer = new ElapsedTime();
            }
            
            // Initialize on first run
            if (!initialized) {
                initialized = true;
                
                // Enable auto-alignment for firing
                robot.camera.enableAutoAlignForFiring();
                
                // Log initial state for debugging
                initialDistance = robot.camera.getStoredDistanceInches();
                lastDetectedTag = robot.camera.getLastDetectedTag();
                
                // Immediately set flywheel to RPM based on stored distance (if available)
                // This lets flywheel spin up while camera is finding the target
                if (initialDistance > 0) {
                    targetRPM = robot.camera.getSuggestedVelocity(1);
                    if (targetRPM > 0) {
                        robot.flywheel.setVelocity(targetRPM);
                    }
                }
                
                // Log initialization info
                if (fireLog != null && label != null) {
                    fireLog.add(String.format("%s INIT: dist=%.0f\" tag=%d rpm=%.0f mode=%s", 
                        label, initialDistance, lastDetectedTag, targetRPM, robot.camera.getScanMode()));
                }
            }
            
            // Always run camera to process detections and update lights
            robot.camera.run();
            robot.flywheel.run();
            
            // Track when we first see a target
            int currentTag = robot.camera.getLastDetectedTag();
            boolean seeingTarget = (currentTag == 2 || currentTag == 3);  // Red=2, Blue=3
            if (seeingTarget && timeFirstTargetSeen < 0) {
                timeFirstTargetSeen = timer.seconds();
                
                // Now that we see a target, get the distance-based RPM
                double distance = robot.camera.getStoredDistanceInches();
                targetRPM = robot.camera.getSuggestedVelocity(1);
                if (targetRPM <= 0) {
                    // Fallback if no distance reading - use provided default
                    targetRPM = defaultRPM;
                }
                robot.flywheel.setVelocity(targetRPM);
                
                if (fireLog != null && label != null) {
                    fireLog.add(String.format("%s TARGET: tag=%d dist=%.0f\" rpm=%.0f", 
                        label, currentTag, distance, targetRPM));
                }
            }
            
            // If we haven't seen a target yet and haven't set RPM, use initial distance or default
            if (targetRPM == 0) {
                // Wait a bit for camera to find target before using fallback
                if (timer.seconds() > SCAN_WAIT_TIME) {
                    double distance = robot.camera.getStoredDistanceInches();
                    targetRPM = robot.camera.getSuggestedVelocity(1);
                    if (targetRPM <= 0) {
                        targetRPM = defaultRPM;  // Use provided default
                    }
                    robot.flywheel.setVelocity(targetRPM);
                    
                    if (fireLog != null && label != null) {
                        fireLog.add(String.format("%s FALLBACK: no target seen, dist=%.0f\" rpm=%.0f", 
                            label, distance, targetRPM));
                    }
                }
            }

            switch (state) {
                case 0:  // Wait for alignment AND flywheel to be ready
                    // Check alignment status
                    boolean aligned = robot.camera.isAligned();
                    if (aligned && timeFirstAligned < 0) {
                        timeFirstAligned = timer.seconds();
                    }
                    
                    // Check flywheel status
                    double currentRPM = robot.flywheel.getCurrentRPM();
                    double flywheelTarget = robot.flywheel.getTargetRPM();
                    boolean atTarget = flywheelTarget > 0 && Math.abs(currentRPM - flywheelTarget) <= RPM_TOLERANCE;
                    
                    if (!atTarget) {
                        stableTimer.reset();
                        timeFirstStable = -1;
                    } else if (timeFirstStable < 0) {
                        timeFirstStable = timer.seconds();
                    }
                    
                    boolean flywheelReady = atTarget && stableTimer.seconds() > STABLE_TIME;
                    boolean timedOut = timer.seconds() > TIMEOUT;
                    
                    // Fire when both ready OR timeout
                    if ((aligned && flywheelReady) || timedOut) {
                        // Log the firing info
                        double firingRPM = robot.flywheel.getCurrentRPM();
                        double firingTarget = robot.flywheel.getTargetRPM();
                        double firingDistance = robot.camera.getStoredDistanceInches();
                        double totalTime = timer.seconds();
                        
                        // Read ball colors from intake and configure kickers
                        configureBallColorsFromIntake();
                        
                        // Set the firing sequence from detected sequence
                        configureSequenceFromCamera();
                        
                        if (fireLog != null && label != null) {
                            String reason = timedOut ? "TIMEOUT" : "ready";
                            String targetInfo = timeFirstTargetSeen >= 0 
                                ? String.format("target@%.2fs", timeFirstTargetSeen)
                                : "NO TARGET!";
                            String alignInfo = timeFirstAligned >= 0 
                                ? String.format("align@%.2fs", timeFirstAligned)
                                : "no align";
                            String rpmInfo = timeFirstStable >= 0 
                                ? String.format("stable@%.2fs", timeFirstStable)
                                : "not stable";
                            fireLog.add(String.format("%s FIRE: actual=%.0f/target=%.0f dist=%.0f\" [%s %s %s] @%.2fs (%s) Seq:%s", 
                                label, firingRPM, firingTarget, firingDistance, 
                                targetInfo, alignInfo, rpmInfo, 
                                totalTime, reason, robot.kickers.getSequence()));
                        }
                        
                        // Start the sequence firing
                        robot.kickers.fireSequence();
                        state = 1;
                    }
                    
                    telemetryPacket.put("Tag Seen", currentTag);
                    telemetryPacket.put("Distance", robot.camera.getStoredDistanceInches());
                    telemetryPacket.put("Aligned", aligned);
                    telemetryPacket.put("Flywheel Target", flywheelTarget);
                    telemetryPacket.put("Flywheel Current", currentRPM);
                    telemetryPacket.put("Flywheel Ready", flywheelReady);
                    telemetryPacket.put("Camera Align", robot.camera.getAlignmentInfo());
                    return true;

                case 1:  // Wait for sequence to complete
                    robot.camera.run();    // Keep camera running for alignment during firing
                    robot.flywheel.run();  // Keep flywheel running for velocity recovery
                    robot.kickers.run();   // Run kicker state machine
                    
                    if (robot.kickers.isSequenceComplete()) {
                        // Disable auto-alignment
                        robot.camera.disableAutoAlignForFiring();
                        state = 2;
                        return false;  // Done
                    }
                    telemetryPacket.put("Sequence", "Firing...");
                    telemetryPacket.put("Flywheel Current", robot.flywheel.getCurrentRPM());
                    return true;

                default:
                    return false;
            }
        }
        
        /**
         * Read ball colors from intake sensors and configure kickers
         */
        private void configureBallColorsFromIntake() {
            org.firstinspires.ftc.teamcode.hardware.Intake.BallColor[] intakeColors = 
                robot.intake.getAllShooterBallColors();
            
            for (int i = 0; i < 3; i++) {
                org.firstinspires.ftc.teamcode.hardware.Kickers.BallColor kickerColor = 
                    convertBallColor(intakeColors[i]);
                robot.kickers.setBallColor(i + 1, kickerColor);
            }
        }
        
        /**
         * Convert Intake.BallColor to Kickers.BallColor
         */
        private org.firstinspires.ftc.teamcode.hardware.Kickers.BallColor convertBallColor(
                org.firstinspires.ftc.teamcode.hardware.Intake.BallColor intakeColor) {
            switch (intakeColor) {
                case GREEN:
                    return org.firstinspires.ftc.teamcode.hardware.Kickers.BallColor.GREEN;
                case PURPLE:
                default:
                    return org.firstinspires.ftc.teamcode.hardware.Kickers.BallColor.PURPLE;
            }
        }
        
        /**
         * Set the kicker firing sequence based on detected camera sequence
         */
        private void configureSequenceFromCamera() {
            org.firstinspires.ftc.teamcode.hardware.Camera.BallSequence detected = 
                robot.camera.getDetectedSequence();
            
            org.firstinspires.ftc.teamcode.hardware.Kickers.Sequence kickerSequence;
            switch (detected) {
                case GPP:
                    kickerSequence = org.firstinspires.ftc.teamcode.hardware.Kickers.Sequence.GPP;
                    break;
                case PGP:
                    kickerSequence = org.firstinspires.ftc.teamcode.hardware.Kickers.Sequence.PGP;
                    break;
                case PPG:
                    kickerSequence = org.firstinspires.ftc.teamcode.hardware.Kickers.Sequence.PPG;
                    break;
                default:
                    kickerSequence = org.firstinspires.ftc.teamcode.hardware.Kickers.Sequence.GPP;
                    break;
            }
            robot.kickers.setSequence(kickerSequence);
        }
    }

    /**
     * Auto-aim using camera, wait for alignment AND flywheel speed, then fire ALL at once.
     * 
     * Similar to AutoAimAndFireSequence but fires all balls at once.
     * Uses ball count from intake to select appropriate RPM table (1/2/3 balls).
     */
    public static class AutoAimAndFireAll implements Action {
        private final RobotHardware robot;
        private final String label;
        private final java.util.List<String> fireLog;
        private ElapsedTime timer;
        private ElapsedTime stableTimer;
        private int state = 0;
        private double timeFirstStable = -1;
        private double timeFirstAligned = -1;
        private boolean initialized = false;
        private double targetRPM = 0;
        private int ballCount = 0;
        private static final double TIMEOUT = 6.0;
        private static final double STABLE_TIME = 0.1;
        private static final double FIRE_HOLD_TIME = 0.5;
        private static final double RPM_TOLERANCE = 75.0;

        public AutoAimAndFireAll(RobotHardware robot) {
            this(robot, null, null);
        }

        public AutoAimAndFireAll(RobotHardware robot, String label, java.util.List<String> fireLog) {
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
            
            if (!initialized) {
                initialized = true;
                
                // Enable auto-alignment for firing
                robot.camera.enableAutoAlignForFiring();
                
                // Get ball count and distance-based RPM
                ballCount = robot.intake.getBallCount();
                if (ballCount <= 0) ballCount = 3;  // Assume full if no detection
                
                targetRPM = robot.camera.getSuggestedVelocity(ballCount);
                if (targetRPM <= 0) {
                    targetRPM = 2600;  // Fallback for 3-ball
                }
                robot.flywheel.setVelocity(targetRPM);
            }

            switch (state) {
                case 0:  // Wait for alignment AND flywheel
                    robot.camera.run();
                    robot.flywheel.run();
                    
                    boolean aligned = robot.camera.isAligned();
                    if (aligned && timeFirstAligned < 0) {
                        timeFirstAligned = timer.seconds();
                    }
                    
                    double currentRPM = robot.flywheel.getCurrentRPM();
                    double flywheelTarget = robot.flywheel.getTargetRPM();
                    boolean atTarget = Math.abs(currentRPM - flywheelTarget) <= RPM_TOLERANCE;
                    
                    if (!atTarget) {
                        stableTimer.reset();
                        timeFirstStable = -1;
                    } else if (timeFirstStable < 0) {
                        timeFirstStable = timer.seconds();
                    }
                    
                    boolean flywheelReady = atTarget && stableTimer.seconds() > STABLE_TIME;
                    boolean timedOut = timer.seconds() > TIMEOUT;
                    
                    if ((aligned && flywheelReady) || timedOut) {
                        double firingRPM = robot.flywheel.getCurrentRPM();
                        double totalTime = timer.seconds();
                        
                        if (fireLog != null && label != null) {
                            String reason = timedOut ? " (TIMEOUT!)" : " (ready)";
                            String alignInfo = timeFirstAligned >= 0 
                                ? String.format("align@%.2fs", timeFirstAligned)
                                : "never aligned";
                            String rpmInfo = timeFirstStable >= 0 
                                ? String.format("rpm@%.2fs", timeFirstStable)
                                : "never stable";
                            fireLog.add(String.format("%s: %.0f RPM, %db [%s, %s] fire@%.2fs%s", 
                                label, firingRPM, ballCount, alignInfo, rpmInfo, totalTime, reason));
                        }
                        
                        robot.kickers.fireAll();
                        timer.reset();
                        state = 1;
                    }
                    
                    telemetryPacket.put("Aligned", aligned);
                    telemetryPacket.put("Flywheel Target", robot.flywheel.getTargetRPM());
                    telemetryPacket.put("Flywheel Current", robot.flywheel.getCurrentRPM());
                    telemetryPacket.put("Flywheel Ready", flywheelReady);
                    return true;

                case 1:  // Wait for kick to complete
                    if (timer.seconds() > FIRE_HOLD_TIME) {
                        robot.kickers.retractAll();
                        robot.camera.disableAutoAlignForFiring();
                        state = 2;
                        return false;
                    }
                    return true;

                default:
                    return false;
            }
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

    /**
     * Log a message to the fire log with timestamp
     */
    public static class LogMessage implements Action {
        private final String message;
        private final java.util.List<String> fireLog;
        private final ElapsedTime autoTimer;

        public LogMessage(String message, java.util.List<String> fireLog, ElapsedTime autoTimer) {
            this.message = message;
            this.fireLog = fireLog;
            this.autoTimer = autoTimer;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (fireLog != null) {
                String timestamp = (autoTimer != null) ? String.format(" %.0fs", autoTimer.seconds()) : "";
                fireLog.add(message + timestamp);
            }
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

    /**
     * Set camera to TeleOp mode (target detection - blue/red goals)
     * Call this before using AutoAimAndFireSequence or AutoAimAndFireAll
     */
    public static class CameraSetTeleOpMode implements Action {
        private final RobotHardware robot;

        public CameraSetTeleOpMode(RobotHardware robot) {
            this.robot = robot;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.camera.setModeTeleOp();
            return false;
        }
    }

    /**
     * Set camera to Pre-Match mode (obelisk sequence detection)
     */
    public static class CameraSetPreMatchMode implements Action {
        private final RobotHardware robot;

        public CameraSetPreMatchMode(RobotHardware robot) {
            this.robot = robot;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.camera.setModePreMatch();
            return false;
        }
    }

    /**
     * Set camera to Demo mode (any tag detection)
     */
    public static class CameraSetDemoMode implements Action {
        private final RobotHardware robot;

        public CameraSetDemoMode(RobotHardware robot) {
            this.robot = robot;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.camera.setModeDemo();
            return false;
        }
    }

    //endregion
}
