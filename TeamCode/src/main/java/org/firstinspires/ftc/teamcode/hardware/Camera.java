package org.firstinspires.ftc.teamcode.hardware;

//region --- Imports ---
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
//endregion

public class Camera
{
    //region --- Constants ---
    //--- Default servo positions (center)
    private static final double YAW_CENTER = 0.5;
    private static final double PITCH_CENTER = 0.65;

    //--- Fine tune increment
    private static final double TUNE_INCREMENT = 0.01;

    //--- AprilTag IDs
    private static final int TAG_RED_TARGET = 2;
    private static final int TAG_BLUE_TARGET = 3;
    private static final int TAG_SEQUENCE_GPP = 1;
    private static final int TAG_SEQUENCE_PGP = 5;
    private static final int TAG_SEQUENCE_PPG = 4;

    //--- HuskyLens screen dimensions (for alignment)
    private static final int SCREEN_CENTER_X = 160;  // 320 / 2
    private static final int SCREEN_CENTER_Y = 120;  // 240 / 2
    private static final int ALIGN_DEADBAND = 20;    // Pixels from center to consider "aligned"
    private static final int ALIGN_SLOWZONE = 60;    // Pixels from center to start slowing down
    private static final double ALIGN_SPEED_MIN = 0.08;  // Minimum rotation speed
    private static final double ALIGN_SPEED_MAX = 0.25;  // Maximum rotation speed
    private static final double ALIGN_SETTLE_TIME = 0.15; // Seconds to wait after reaching deadband

    //--- Distance estimation constants
    //--- Formula: distance = (realSize * focalLength) / pixelSize
    //--- Calibrated from measurements: 36"@55px, 60"@33px, 96"@21px → avg focal length 306
    private static final double TAG_REAL_SIZE_INCHES = 6.5;  // Physical AprilTag size
    private static final double HUSKY_FOCAL_LENGTH = 306.0;  // Calibrated focal length

    //--- Velocity suggestion based on distance (linear interpolation)
    private static final double VELOCITY_NEAR_DISTANCE = 36.0;   // 3 feet in inches
    private static final double VELOCITY_FAR_DISTANCE = 120.0;   // 10 feet in inches
    private static final double VELOCITY_MIN_RPM = 1500.0;       // RPM at near distance
    private static final double VELOCITY_MAX_RPM = 4000.0;       // RPM at far distance

    //--- Pitch scanning constants
    private static final double PITCH_SCAN_MIN = 0.50;   // Lowest pitch (looking DOWN toward floor)
    private static final double PITCH_SCAN_MAX = 0.75;   // Highest pitch (looking UP toward ceiling)
    private static final double PITCH_SCAN_STEP = 0.01;  // Step size for scanning (smaller = smoother)
    private static final double PITCH_DEADBAND = 30;     // Pixels from Y center to consider "centered"

    //--- Pre-match camera position (fixed, no scanning)
    private static final double PREMATCH_YAW = 0.5;      // Yaw for pre-match (adjust as needed)
    private static final double PREMATCH_PITCH = 0.70;   // Pitch for pre-match obelisk viewing

    //--- Tag heights from floor (inches) - for reference
    //--- Goal tags: 38.75" - 9.25" = 29.5" center height
    //--- Obelisk tags: 23" - 3.75" = 19.25" center height
    //--- Camera height: 17" from floor
    //endregion

    //region --- Enums ---
    public enum ScanMode
    {
        PRE_MATCH,  // Obelisk only (sequence tags, ignore goal targets)
        TELEOP,     // Goals only (ignore obelisk/sequence tags)
        DEMO        // Any tag triggers (for testing/demo)
    }
    //endregion

    //region --- Hardware ---
    private final HuskyLens _huskyLens;
    private final Servo _servoYaw;
    private final Servo _servoPitch;
    private final Kickers _kickers;
    private final Lights _lights;
    private final Drive _drive;
    private final Gamepad _gamepad1;
    private final Gamepad _gamepad2;
    private final Telemetry _telemetry;
    private final boolean _showInfo;
    private final int _robotVersion;
    //endregion

    //region --- State ---
    private boolean _isConnected = false;
    private int _lastDetectedTag = -1;
    private HuskyLens.Block[] _blocks = new HuskyLens.Block[0];
    private HuskyLens.Algorithm _currentAlgorithm = HuskyLens.Algorithm.TAG_RECOGNITION;

    //--- Fine tune state
    private double _tuneYaw = YAW_CENTER;
    private double _tunePitch = PITCH_CENTER;
    private boolean _tuneInitialized = false;

    //--- Tag detection state (to prevent repeated triggers)
    private int _lastProcessedTag = -1;

    //--- Alignment lock state
    private boolean _alignmentLockEnabled = false;  // Manual lock via Y button
    private boolean _autoAlignForFiring = false;    // Auto-align when firing at target
    private boolean _isAligned = false;             // Currently within deadband
    private boolean _yButtonPressed = false;        // Debounce Y button
    private boolean _aButtonPressed = false;        // Debounce A button
    private ElapsedTime _alignSettleTimer = new ElapsedTime();  // Timer for settling
    private boolean _isSettling = false;            // Currently in settling period

    //--- Distance estimation state
    private double _lastDistanceInches = -1.0;      // Last calculated distance (-1 if no tag)

    //--- Scan mode state
    private ScanMode _scanMode = ScanMode.DEMO;     // Current operating mode
    private boolean _isScanning = false;            // Currently scanning for tags
    private boolean _scanDirectionUp = true;        // Scan direction (true = increasing pitch)
    private double _currentScanPitch = PITCH_CENTER; // Current pitch during scan
    private ElapsedTime _scanTimer = new ElapsedTime(); // Timer for scan steps
    private static final double SCAN_STEP_TIME = 0.15;  // Time between scan steps (seconds)
    //endregion

    //region --- Constructor ---
    public Camera(
            HuskyLens huskyLens,
            Servo servoYaw,
            Servo servoPitch,
            Kickers kickers,
            Lights lights,
            Drive drive,
            Gamepad gamepad1,
            Gamepad gamepad2,
            Telemetry telemetry,
            int robotVersion,
            boolean showInfo
    )
    {
        this._huskyLens = huskyLens;
        this._servoYaw = servoYaw;
        this._servoPitch = servoPitch;
        this._kickers = kickers;
        this._lights = lights;
        this._drive = drive;
        this._gamepad1 = gamepad1;
        this._gamepad2 = gamepad2;
        this._telemetry = telemetry;
        this._robotVersion = robotVersion;
        this._showInfo = showInfo;
    }
    //endregion

    //region --- Initialize ---
    public void initialize()
    {
        //--- Center the camera
        _servoYaw.setPosition(YAW_CENTER);
        _servoPitch.setPosition(PITCH_CENTER);
        _tuneYaw = YAW_CENTER;
        _tunePitch = PITCH_CENTER;

        //--- Check if HuskyLens is connected
        _isConnected = _huskyLens.knock();

        //--- Always try to set algorithm mode (even if knock fails)
        _huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }
    //endregion

    //region --- Run (call this in your main loop) ---
    public void run()
    {
        //--- Try to read even if knock failed (sometimes knock fails but camera works)
        try
        {
            //--- Read detected blocks from HuskyLens
            _blocks = _huskyLens.blocks();

            //--- Filter blocks based on current scan mode
            HuskyLens.Block targetBlock = findTargetBlock();

            //--- Update distance estimation
            updateDistance();

            //--- Process detected AprilTags
            if (targetBlock != null)
            {
                //--- Stop scanning - we found a valid tag
                _isScanning = false;

                int detectedId = targetBlock.id;
                _lastDetectedTag = detectedId;

                //--- Adjust pitch to keep tag centered vertically
                adjustPitchToTrack(targetBlock);

                //--- Only process if it's a new tag (prevent repeated triggers)
                if (detectedId != _lastProcessedTag)
                {
                    _lastProcessedTag = detectedId;
                    processAprilTag(detectedId);
                }
            }
            else
            {
                //--- No valid tags detected - start scanning
                if (_lastDetectedTag != -1)
                {
                    //--- Just lost sight of tag, turn off lights
                    _lights.setAllOff();
                }
                _lastDetectedTag = -1;
                _lastProcessedTag = -1;

                //--- Scan for tags (only in TELEOP and DEMO modes)
                if (_scanMode != ScanMode.PRE_MATCH)
                {
                    runPitchScan();
                }
            }
        }
        catch (Exception e)
        {
            //--- HuskyLens communication error
            _isConnected = false;
        }
    }

    //--- Find the best target block based on current scan mode
    private HuskyLens.Block findTargetBlock()
    {
        if (_blocks.length == 0) return null;

        for (HuskyLens.Block block : _blocks)
        {
            if (isValidTagForMode(block.id))
            {
                return block;
            }
        }
        return null;
    }

    //--- Check if a tag ID is valid for the current scan mode
    private boolean isValidTagForMode(int tagId)
    {
        switch (_scanMode)
        {
            case PRE_MATCH:
                //--- Only sequence tags (obelisk)
                return tagId == TAG_SEQUENCE_GPP || 
                       tagId == TAG_SEQUENCE_PGP || 
                       tagId == TAG_SEQUENCE_PPG;

            case TELEOP:
                //--- Only goal tags
                return tagId == TAG_BLUE_TARGET || tagId == TAG_RED_TARGET;

            case DEMO:
            default:
                //--- Any tag is valid
                return tagId == TAG_BLUE_TARGET || 
                       tagId == TAG_RED_TARGET ||
                       tagId == TAG_SEQUENCE_GPP || 
                       tagId == TAG_SEQUENCE_PGP || 
                       tagId == TAG_SEQUENCE_PPG;
        }
    }

    //--- Adjust pitch to keep the detected tag vertically centered
    private void adjustPitchToTrack(HuskyLens.Block block)
    {
        int errorY = block.y - SCREEN_CENTER_Y;
        
        //--- Only adjust if outside deadband
        if (Math.abs(errorY) > PITCH_DEADBAND)
        {
            //--- Positive errorY means tag is below center, need to DECREASE pitch (look down)
            //--- Negative errorY means tag is above center, need to INCREASE pitch (look up)
            double adjustment = (errorY > 0) ? -PITCH_SCAN_STEP : PITCH_SCAN_STEP;
            double newPitch = _currentScanPitch + adjustment;
            
            //--- Clamp to scan range to prevent tracking beyond limits
            newPitch = Math.max(PITCH_SCAN_MIN, Math.min(PITCH_SCAN_MAX, newPitch));
            
            _currentScanPitch = newPitch;
            _servoPitch.setPosition(_currentScanPitch);
            _tunePitch = _currentScanPitch;
        }
    }

    //--- Run pitch scanning to find tags
    private void runPitchScan()
    {
        //--- Start scanning if not already
        if (!_isScanning)
        {
            _isScanning = true;
            _scanTimer.reset();
            return;
        }

        //--- Only step at defined intervals
        if (_scanTimer.seconds() < SCAN_STEP_TIME) return;
        _scanTimer.reset();

        //--- Move pitch in current direction
        if (_scanDirectionUp)
        {
            _currentScanPitch += PITCH_SCAN_STEP;
            if (_currentScanPitch >= PITCH_SCAN_MAX)
            {
                _currentScanPitch = PITCH_SCAN_MAX;
                _scanDirectionUp = false;
            }
        }
        else
        {
            _currentScanPitch -= PITCH_SCAN_STEP;
            if (_currentScanPitch <= PITCH_SCAN_MIN)
            {
                _currentScanPitch = PITCH_SCAN_MIN;
                _scanDirectionUp = true;
            }
        }

        _servoPitch.setPosition(_currentScanPitch);
        _tunePitch = _currentScanPitch;
    }

    //--- Process detected AprilTag and update kickers/lights
    private void processAprilTag(int tagId)
    {
        switch (tagId)
        {
            case TAG_BLUE_TARGET:
                //--- Blue target: solid blue lights
                _lights.setAll(Lights.Color.BLUE);
                break;

            case TAG_SEQUENCE_GPP:
                //--- GPP sequence: set kickers to GPP, solid Green-Purple-Purple
                _kickers.setSequence(Kickers.Sequence.GPP);
                _lights.setLeft(Lights.Color.GREEN);
                _lights.setMiddle(Lights.Color.PURPLE);
                _lights.setRight(Lights.Color.PURPLE);
                break;

            case TAG_SEQUENCE_PGP:
                //--- PGP sequence: set kickers to PGP, solid Purple-Green-Purple
                _kickers.setSequence(Kickers.Sequence.PGP);
                _lights.setLeft(Lights.Color.PURPLE);
                _lights.setMiddle(Lights.Color.GREEN);
                _lights.setRight(Lights.Color.PURPLE);
                break;

            case TAG_SEQUENCE_PPG:
                //--- PPG sequence: set kickers to PPG, solid Purple-Purple-Green
                _kickers.setSequence(Kickers.Sequence.PPG);
                _lights.setLeft(Lights.Color.PURPLE);
                _lights.setMiddle(Lights.Color.PURPLE);
                _lights.setRight(Lights.Color.GREEN);
                break;

            case TAG_RED_TARGET:
                //--- Red target: solid red lights
                _lights.setAll(Lights.Color.RED);
                break;
        }
    }
    //endregion

    //region --- Public Methods - Camera Position ---

    //--- Set camera yaw position (0.0 to 1.0)
    public void setYaw(double position)
    {
        position = clamp(position);
        _servoYaw.setPosition(position);
        _tuneYaw = position;
    }

    //--- Set camera pitch position (0.0 to 1.0)
    public void setPitch(double position)
    {
        position = clamp(position);
        _servoPitch.setPosition(position);
        _tunePitch = position;
    }

    //--- Set both yaw and pitch at once
    public void setPosition(double yaw, double pitch)
    {
        setYaw(yaw);
        setPitch(pitch);
    }

    //--- Center the camera
    public void center()
    {
        setPosition(YAW_CENTER, PITCH_CENTER);
    }

    //--- Get current yaw position
    public double getYaw()
    {
        return _servoYaw.getPosition();
    }

    //--- Get current pitch position
    public double getPitch()
    {
        return _servoPitch.getPosition();
    }

    //endregion

    //region --- Public Methods - Scan Mode ---

    //--- Set the scan mode (PRE_MATCH, TELEOP, DEMO)
    public void setScanMode(ScanMode mode)
    {
        _scanMode = mode;
        _lastProcessedTag = -1;  // Allow re-triggering when mode changes
        
        if (mode == ScanMode.PRE_MATCH)
        {
            //--- Pre-match: set camera to fixed position, no scanning
            _isScanning = false;
            setPosition(PREMATCH_YAW, PREMATCH_PITCH);
            _currentScanPitch = PREMATCH_PITCH;
        }
        else
        {
            //--- TELEOP/DEMO: start at scan maximum (looking UP toward goals) and scan downward
            setPosition(YAW_CENTER, PITCH_SCAN_MAX);
            _currentScanPitch = PITCH_SCAN_MAX;
            _scanDirectionUp = false;  // Will decrease pitch (scan downward toward floor)
            _isScanning = false;  // Will start on next run() if no tag found
        }
    }

    //--- Get the current scan mode
    public ScanMode getScanMode()
    {
        return _scanMode;
    }

    //--- Convenience methods for setting scan mode
    public void setModePreMatch()
    {
        setScanMode(ScanMode.PRE_MATCH);
    }

    public void setModeTeleOp()
    {
        setScanMode(ScanMode.TELEOP);
    }

    public void setModeDemo()
    {
        setScanMode(ScanMode.DEMO);
    }

    //--- Check if currently scanning for tags
    public boolean isScanning()
    {
        return _isScanning;
    }

    //--- Set the pre-match camera position (for viewing obelisk)
    //--- Call this during init to adjust where camera looks for sequence tags
    public void setPreMatchPosition(double yaw, double pitch)
    {
        if (_scanMode == ScanMode.PRE_MATCH)
        {
            setPosition(yaw, pitch);
            _currentScanPitch = pitch;
        }
    }

    //endregion

    //region --- Public Methods - Detection Info ---

    //--- Check if HuskyLens is connected
    public boolean isConnected()
    {
        return _isConnected;
    }

    //--- Get the last detected AprilTag ID (-1 if none)
    public int getLastDetectedTag()
    {
        return _lastDetectedTag;
    }

    //--- Get all detected blocks
    public HuskyLens.Block[] getBlocks()
    {
        return _blocks;
    }

    //--- Get the number of detected blocks
    public int getBlockCount()
    {
        return _blocks.length;
    }

    //--- Reset the last processed tag (allows re-triggering on same tag)
    public void resetLastProcessedTag()
    {
        _lastProcessedTag = -1;
    }

    //endregion

    //region --- Public Methods - Distance Estimation ---

    //--- Get the estimated distance to the AprilTag in inches (-1 if no tag detected)
    public double getDistanceInches()
    {
        return _lastDistanceInches;
    }

    //--- Get distance formatted as inches string
    public String getDistanceFormatted()
    {
        if (_lastDistanceInches < 0) return "No tag";
        
        int inches = (int) Math.round(_lastDistanceInches);
        return String.format("%d in", inches);
    }

    //--- Calculate distance from the detected AprilTag
    //--- Uses the formula: distance = (realSize * focalLength) / pixelSize
    private void updateDistance()
    {
        if (_blocks.length == 0)
        {
            _lastDistanceInches = -1.0;
            return;
        }

        //--- Use the width of the tag (more reliable than height for distance)
        int pixelWidth = _blocks[0].width;
        
        if (pixelWidth <= 0)
        {
            _lastDistanceInches = -1.0;
            return;
        }

        //--- Calculate distance using similar triangles
        _lastDistanceInches = (TAG_REAL_SIZE_INCHES * HUSKY_FOCAL_LENGTH) / pixelWidth;
    }

    //--- Get suggested flywheel velocity based on distance (returns -1 if no tag)
    //--- Uses linear interpolation between near/far distances
    public double getSuggestedVelocity()
    {
        if (_lastDistanceInches < 0) return -1.0;
        
        //--- Clamp distance to valid range
        double clampedDistance = Math.max(VELOCITY_NEAR_DISTANCE, 
                                          Math.min(VELOCITY_FAR_DISTANCE, _lastDistanceInches));
        
        //--- Linear interpolation: velocity = min + (max - min) * ((distance - near) / (far - near))
        double ratio = (clampedDistance - VELOCITY_NEAR_DISTANCE) / 
                       (VELOCITY_FAR_DISTANCE - VELOCITY_NEAR_DISTANCE);
        double suggestedVelocity = VELOCITY_MIN_RPM + (VELOCITY_MAX_RPM - VELOCITY_MIN_RPM) * ratio;
        
        return suggestedVelocity;
    }

    //endregion

    //region --- Public Methods - Robot Alignment ---

    //--- Enable alignment lock (Y button or called programmatically)
    public void enableAlignmentLock()
    {
        _alignmentLockEnabled = true;
        _drive.setBrakeMode(true);  // Enable brake mode to resist being pushed
    }

    //--- Disable alignment lock (A button or called programmatically)
    public void disableAlignmentLock()
    {
        _alignmentLockEnabled = false;
        _autoAlignForFiring = false;
        _isAligned = false;
        _drive.setBrakeMode(false);  // Return to float mode
        _drive.stopMotors();
    }

    //--- Enable auto-alignment for firing (called by Kickers when firing at target)
    public void enableAutoAlignForFiring()
    {
        _autoAlignForFiring = true;
        _drive.setBrakeMode(true);
    }

    //--- Disable auto-alignment after firing sequence complete
    public void disableAutoAlignForFiring()
    {
        _autoAlignForFiring = false;
        if (!_alignmentLockEnabled)
        {
            _drive.setBrakeMode(false);
        }
        _drive.stopMotors();
    }

    //--- Check if we're currently aligned (within deadband)
    public boolean isAligned()
    {
        return _isAligned;
    }

    //--- Check if alignment lock is enabled (manual or auto)
    public boolean isAlignmentActive()
    {
        return _alignmentLockEnabled || _autoAlignForFiring;
    }

    //--- Check if we're looking at a target (blue or red)
    public boolean isLookingAtTarget()
    {
        return _lastDetectedTag == TAG_BLUE_TARGET || _lastDetectedTag == TAG_RED_TARGET;
    }

    //--- Run alignment logic - call this every loop when alignment is active
    private void runAlignment()
    {
        if (_blocks.length == 0)
        {
            _isAligned = false;
            _isSettling = false;
            _drive.stopMotors();
            return;
        }

        //--- Get the X position of the first detected block
        int blockX = _blocks[0].x;
        int errorX = blockX - SCREEN_CENTER_X;
        int absError = Math.abs(errorX);

        //--- Check if we're within the deadband (aligned)
        if (absError <= ALIGN_DEADBAND)
        {
            //--- Start settling timer if not already settling
            if (!_isSettling)
            {
                _isSettling = true;
                _alignSettleTimer.reset();
            }
            
            //--- Check if we've been settled long enough
            if (_alignSettleTimer.seconds() >= ALIGN_SETTLE_TIME)
            {
                _isAligned = true;
            }
            
            //--- Keep motors stopped while settling
            _drive.stopMotors();
        }
        else
        {
            //--- Outside deadband - need to rotate
            _isAligned = false;
            _isSettling = false;

            //--- Calculate speed: minimum in slow zone, proportional outside
            double speed;
            if (absError <= ALIGN_SLOWZONE)
            {
                speed = ALIGN_SPEED_MIN;
            }
            else
            {
                double proportion = (double) absError / 160.0;
                speed = ALIGN_SPEED_MIN + (proportion * (ALIGN_SPEED_MAX - ALIGN_SPEED_MIN));
                speed = Math.min(speed, ALIGN_SPEED_MAX);
            }

            //--- Rotate toward center
            rotateToward(errorX, speed);
        }
    }

    //--- Helper: Rotate in the direction to reduce error
    private void rotateToward(int errorX, double speed)
    {
        if (errorX > 0)
        {
            _drive.rotateRight(speed);
        }
        else
        {
            _drive.rotateLeft(speed);
        }
    }

    //--- Handle alignment controls (Y to enable, A to disable)
    public void handleAlignmentControls()
    {
        //--- Y button - enable alignment lock
        if (_gamepad2.y)
        {
            if (!_yButtonPressed)
            {
                _yButtonPressed = true;
                enableAlignmentLock();
            }
        }
        else
        {
            _yButtonPressed = false;
        }

        //--- A button - disable alignment lock
        if (_gamepad2.a)
        {
            if (!_aButtonPressed)
            {
                _aButtonPressed = true;
                disableAlignmentLock();
            }
        }
        else
        {
            _aButtonPressed = false;
        }

        //--- Run alignment if enabled
        if (_alignmentLockEnabled || _autoAlignForFiring)
        {
            runAlignment();
        }
    }

    //endregion

    //region --- Public Methods - Algorithm/Mode Selection ---

    //--- Set the HuskyLens algorithm mode
    public void setAlgorithm(HuskyLens.Algorithm algorithm)
    {
        _currentAlgorithm = algorithm;
        _huskyLens.selectAlgorithm(algorithm);
    }

    //--- Get the current algorithm mode
    public HuskyLens.Algorithm getAlgorithm()
    {
        return _currentAlgorithm;
    }

    //--- Convenience methods for common modes
    public void setModeTagRecognition()
    {
        setAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    public void setModeObjectRecognition()
    {
        setAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION);
    }

    public void setModeColorRecognition()
    {
        setAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    public void setModeObjectTracking()
    {
        setAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
    }

    public void setModeObjectClassification()
    {
        setAlgorithm(HuskyLens.Algorithm.OBJECT_CLASSIFICATION);
    }

    public void setModeLineTracking()
    {
        setAlgorithm(HuskyLens.Algorithm.LINE_TRACKING);
    }

    //endregion

    //region --- Fine Tune Mode ---

    //--- Fine tune camera position using gamepad2
    //--- Dpad Up/Down: Pitch
    //--- Dpad Left/Right: Yaw
    //--- Y: Enable alignment lock
    //--- A: Disable alignment lock
    public void fineTuneCameraPos()
    {
        //--- Initialize positions on first call
        if (!_tuneInitialized)
        {
            _tuneInitialized = true;
            _tuneYaw = _servoYaw.getPosition();
            _tunePitch = _servoPitch.getPosition();
        }

        //--- Handle alignment controls (Y to enable, A to disable)
        handleAlignmentControls();

        //--- Dpad Up - pitch up
        if (_gamepad2.dpad_up)
        {
            _tunePitch = clamp(_tunePitch + TUNE_INCREMENT);
            _servoPitch.setPosition(_tunePitch);
        }

        //--- Dpad Down - pitch down
        if (_gamepad2.dpad_down)
        {
            _tunePitch = clamp(_tunePitch - TUNE_INCREMENT);
            _servoPitch.setPosition(_tunePitch);
        }

        //--- Dpad Left - yaw left
        if (_gamepad2.dpad_left)
        {
            _tuneYaw = clamp(_tuneYaw - TUNE_INCREMENT);
            _servoYaw.setPosition(_tuneYaw);
        }

        //--- Dpad Right - yaw right
        if (_gamepad2.dpad_right)
        {
            _tuneYaw = clamp(_tuneYaw + TUNE_INCREMENT);
            _servoYaw.setPosition(_tuneYaw);
        }

        //--- Display telemetry
        _telemetry.addData("--- CAMERA FINE TUNE ---", "");
        _telemetry.addData("Pitch", "Dpad Up/Down");
        _telemetry.addData("Yaw", "Dpad Left/Right");
        _telemetry.addData("Y Button", "ENABLE Alignment Lock");
        _telemetry.addData("A Button", "DISABLE Alignment Lock");
        _telemetry.addData("------------------------", "");
        _telemetry.addData("Yaw", "%.3f", _tuneYaw);
        _telemetry.addData("Pitch", "%.3f", _tunePitch);
        _telemetry.addData("Scan Mode", _scanMode);
        _telemetry.addData("Scanning", _isScanning ? "YES" : "No");
        _telemetry.addData("------------------------", "");
        _telemetry.addData("Connected", _isConnected);
        _telemetry.addData("Block Count", _blocks.length);
        _telemetry.addData("Last Tag ID", _lastDetectedTag);
        _telemetry.addData("Alignment Lock", _alignmentLockEnabled ? "ENABLED" : "Off");
        _telemetry.addData("Auto-Align Fire", _autoAlignForFiring ? "ACTIVE" : "Off");
        _telemetry.addData("Is Aligned", _isAligned ? "YES" : "No");
        if (_blocks.length > 0)
        {
            int errorX = _blocks[0].x - SCREEN_CENTER_X;
            _telemetry.addData("Align Error", "%d px (%s)", errorX, 
                    Math.abs(errorX) <= ALIGN_DEADBAND ? "ALIGNED" : (errorX > 0 ? "RIGHT" : "LEFT"));
            _telemetry.addData("Distance", getDistanceFormatted());
            _telemetry.addData("Suggested Velocity", "%.0f RPM", getSuggestedVelocity());
        }
        
        //--- Show all detected blocks
        for (int i = 0; i < _blocks.length; i++)
        {
            _telemetry.addData("Block " + i, "ID=%d x=%d y=%d w=%d h=%d", 
                    _blocks[i].id, _blocks[i].x, _blocks[i].y, 
                    _blocks[i].width, _blocks[i].height);
        }
    }

    //endregion

    //region --- Telemetry ---

    public void getTelemetry()
    {
        if (_showInfo)
        {
            _telemetry.addData("Camera Connected", _isConnected);
            _telemetry.addData("Camera Yaw", "%.2f", _servoYaw.getPosition());
            _telemetry.addData("Camera Pitch", "%.2f", _servoPitch.getPosition());
            _telemetry.addData("Scan Mode", _scanMode);
            _telemetry.addData("Scanning", _isScanning);
            _telemetry.addData("Last Tag", _lastDetectedTag);
            _telemetry.addData("Blocks Detected", _blocks.length);
            _telemetry.addData("Distance", getDistanceFormatted());
            if (_lastDistanceInches > 0)
            {
                _telemetry.addData("Suggested Velocity", "%.0f RPM", getSuggestedVelocity());
            }
        }
    }

    //endregion

    //region --- Utility Methods ---

    //--- Clamp value between 0 and 1
    private double clamp(double value)
    {
        return Math.max(0.0, Math.min(1.0, value));
    }

    //endregion
}
