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
    private static final double ALIGN_SPEED_MIN = 0.20; // 0.08;  // Minimum rotation speed (must overcome friction)
    private static final double ALIGN_SPEED_MAX = 0.50; // 0.25;  // Maximum rotation speed
    private static final double ALIGN_SETTLE_TIME = 0.15; // Seconds to wait after reaching deadband

    //--- Distance estimation constants
    //--- Formula: distance = (realSize * focalLength) / pixelSize
    //--- Calibrated from measurements: 36"@55px, 60"@33px, 96"@21px → avg focal length 306
    private static final double TAG_REAL_SIZE_INCHES = 6.5;  // Physical AprilTag size
    private static final double HUSKY_FOCAL_LENGTH = 306.0;  // Calibrated focal length

    //--- RPM lookup tables by distance (index 0 = 1 inch, index 119 = 120 inches)
    //--- When firing all balls at once, flywheel slows down so we need higher initial RPM
    //--- Each table has 120 entries, one per inch from 1" to 120"
    //--- TODO: Tune these values based on actual shooting tests
    private static final double[] RPM_TABLE_1_BALL = {
        //--- 1-10 inches (very close, shouldn't happen in practice)
        2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000,
        //--- 11-20 inches
        2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000,
        //--- 21-30 inches (close shots)
        2050, 2100, 2150, 2200, 2250, 2300, 2350, 2400, 2450, 2500,
        //--- 31-40 inches
        2525, 2550, 2575, 2600, 2625, 2650, 2675, 2700, 2725, 2750,
        //--- 41-50 inches
        2775, 2800, 2825, 2850, 2875, 2900, 2925, 2950, 2975, 3000,
        //--- 51-60 inches
        3000, 3025, 3050, 3075, 3100, 3125, 3150, 3175, 3200, 3200,
        //--- 61-70 inches
        3200, 3225, 3250, 3275, 3300, 3325, 3350, 3375, 3400, 3400,
        //--- 71-80 inches
        3400, 3425, 3450, 3475, 3500, 3525, 3550, 3575, 3600, 3600,
        //--- 81-90 inches
        3600, 3625, 3650, 3675, 3700, 3725, 3750, 3775, 3800, 3800,
        //--- 91-100 inches
        3800, 3825, 3850, 3875, 3900, 3925, 3950, 3975, 4000, 4000,
        //--- 101-110 inches
        4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000,
        //--- 111-120 inches
        4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000
    };
    
    private static final double[] RPM_TABLE_2_BALLS = {
        //--- 1-10 inches (very close, shouldn't happen in practice)
        2300, 2300, 2300, 2300, 2300, 2300, 2300, 2300, 2300, 2300,
        //--- 11-20 inches
        2300, 2300, 2300, 2300, 2300, 2300, 2300, 2300, 2300, 2300,
        //--- 21-30 inches (close shots)
        2350, 2400, 2450, 2500, 2550, 2600, 2650, 2700, 2750, 2800,
        //--- 31-40 inches
        2825, 2850, 2875, 2900, 2925, 2950, 2975, 3000, 3025, 3050,
        //--- 41-50 inches
        3075, 3100, 3125, 3150, 3175, 3200, 3225, 3250, 3275, 3300,
        //--- 51-60 inches
        3300, 3325, 3350, 3375, 3400, 3425, 3450, 3475, 3500, 3500,
        //--- 61-70 inches
        3500, 3525, 3550, 3575, 3600, 3625, 3650, 3675, 3700, 3700,
        //--- 71-80 inches
        3700, 3725, 3750, 3775, 3800, 3825, 3850, 3875, 3900, 3900,
        //--- 81-90 inches
        3900, 3925, 3950, 3975, 4000, 4000, 4000, 4000, 4000, 4000,
        //--- 91-100 inches
        4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000,
        //--- 101-110 inches
        4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000,
        //--- 111-120 inches
        4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000
    };
    
    private static final double[] RPM_TABLE_3_BALLS = {
        //--- 1-10 inches (very close, shouldn't happen in practice)
        2600, 2600, 2600, 2600, 2600, 2600, 2600, 2600, 2600, 2600,
        //--- 11-20 inches
        2600, 2600, 2600, 2600, 2600, 2600, 2600, 2600, 2600, 2600,
        //--- 21-30 inches (close shots)
        2650, 2700, 2750, 2800, 2850, 2900, 2950, 3000, 3050, 3100,
        //--- 31-40 inches
        3125, 3150, 3175, 3200, 3225, 3250, 3275, 3300, 3325, 3350,
        //--- 41-50 inches
        3375, 3400, 3425, 3450, 3475, 3500, 3525, 3550, 3575, 3600,
        //--- 51-60 inches
        3600, 3625, 3650, 3675, 3700, 3725, 3750, 3775, 3800, 3800,
        //--- 61-70 inches
        3800, 3825, 3850, 3875, 3900, 3925, 3950, 3975, 4000, 4000,
        //--- 71-80 inches
        4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000,
        //--- 81-90 inches
        4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000,
        //--- 91-100 inches
        4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000,
        //--- 101-110 inches
        4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000,
        //--- 111-120 inches
        4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000
    };

    //--- Velocity adjustment state (for manual tuning with Y/A buttons)
    private static final double VELOCITY_FLOOR = 2000.0;         // Absolute minimum RPM allowed
    private static final double VELOCITY_CEILING = 4000.0;       // Absolute maximum RPM allowed
    private static final double VELOCITY_INCREMENT = 50.0;       // RPM change per button press

    //--- Fixed distance presets for manual override (inches)
    //--- These are used when camera isn't working or for known shooting positions
    //--- Light pattern shows how many are locked: SHORT=1, MEDIUM=2, LONG=3 orange lights
    public static final double FIXED_DISTANCE_SHORT = 26.0;    // Short distance preset
    public static final double FIXED_DISTANCE_MEDIUM = 36.0;   // Medium distance preset
    public static final double FIXED_DISTANCE_LONG = 110.0;    // Long distance preset

    //--- Pitch scanning constants
    private static final double PITCH_SCAN_MIN = 0.60;   // Lowest pitch (looking DOWN toward floor)
    private static final double PITCH_SCAN_MAX = 0.75;   // Highest pitch (looking UP toward ceiling)
    private static final double PITCH_SCAN_STEP = 0.01;  // Step size for scanning (smaller = smoother)
    private static final double PITCH_DEADBAND = 30;     // Pixels from Y center to consider "centered"

    //--- Light hold time (debounce) - lights stay on this long after losing tracking
    private static final double LIGHT_HOLD_TIME = 0.5;   // Seconds to hold lights after losing tag

    //--- Distance smoothing constants
    private static final double EMA_ALPHA = 0.3;         // EMA weight (0.3 = 30% new, 70% old) - higher = more responsive
    private static final int SMA_SAMPLE_COUNT = 5;       // Number of samples for simple moving average

    //--- Pre-match camera position (default, can be overridden)
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

    //--- Targeting mode for gp2 controls
    public enum TargetingMode
    {
        AUTO_AIM,       // Camera auto-align: Y=lock on, A=release
        MANUAL_TARGET   // Manual distance presets: Y=Short, B=Medium, A=Long
    }

    //--- Ball sequence patterns (detected from obelisk AprilTags)
    public enum BallSequence
    {
        UNKNOWN,    // Not yet detected
        GPP,        // Green, Purple, Purple
        PGP,        // Purple, Green, Purple
        PPG         // Purple, Purple, Green
    }

    //--- Distance lock type for manual override
    //--- Light pattern: SHORT=left orange, MEDIUM=left+center orange, LONG=all orange
    public enum DistanceLockType
    {
        NONE,   // Camera updates distance normally
        SHORT,  // Locked to FIXED_DISTANCE_SHORT
        MEDIUM, // Locked to FIXED_DISTANCE_MEDIUM
        LONG    // Locked to FIXED_DISTANCE_LONG
    }

    //--- Distance smoothing mode
    public enum SmoothingMode
    {
        NONE,   // No smoothing - raw readings
        EMA,    // Exponential moving average (default) - responsive with smoothing
        SMA     // Simple moving average - 5 samples, more lag but very stable
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
    private BallSequence _detectedSequence = BallSequence.UNKNOWN;
    private HuskyLens.Block[] _blocks = new HuskyLens.Block[0];
    private HuskyLens.Algorithm _currentAlgorithm = HuskyLens.Algorithm.TAG_RECOGNITION;

    //--- Targeting mode state (gp2 dpad up/down to switch)
    private TargetingMode _targetingMode = TargetingMode.AUTO_AIM;  // Default to auto-aim
    private boolean _dpadUpPressed = false;     // Debounce dpad up
    private boolean _dpadDownPressed = false;   // Debounce dpad down
    private boolean _bButtonPressed = false;    // Debounce B button

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
    private double _lastDistanceInches = -1.0;      // Current distance (-1 if no tag visible)
    private double _storedDistanceInches = -1.0;    // Last good distance (persists when tag lost)
    private double _velocityAdjustment = 0.0;       // Manual adjustment to RPM (added to lookup value)
    private DistanceLockType _distanceLockType = DistanceLockType.NONE;  // Distance lock state

    //--- Distance smoothing state
    private SmoothingMode _smoothingMode = SmoothingMode.EMA;  // Default to EMA
    private double _emaDistance = -1.0;             // Current EMA value
    private double[] _smaBuffer = new double[SMA_SAMPLE_COUNT];  // Circular buffer for SMA
    private int _smaIndex = 0;                      // Current index in SMA buffer
    private int _smaCount = 0;                      // Number of valid samples in SMA buffer

    //--- Scan mode state
    private ScanMode _scanMode = ScanMode.DEMO;     // Current operating mode
    private boolean _isScanning = false;            // Currently scanning for tags
    private boolean _scanDirectionUp = true;        // Scan direction (true = increasing pitch)
    private double _currentScanPitch = PITCH_CENTER; // Current pitch during scan
    private ElapsedTime _scanTimer = new ElapsedTime(); // Timer for scan steps
    private static final double SCAN_STEP_TIME = 0.15;  // Time between scan steps (seconds)

    //--- Light hold timer (debounce to prevent flickering)
    private ElapsedTime _lightHoldTimer = new ElapsedTime();
    private boolean _lightHoldActive = false;       // True when we just lost a tag and are holding lights
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
                
                //--- Cancel any light hold timer since we're tracking again
                _lightHoldActive = false;

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
                    //--- Just lost sight of tag, start hold timer instead of turning off immediately
                    if (!_lightHoldActive)
                    {
                        _lightHoldActive = true;
                        _lightHoldTimer.reset();
                    }
                }
                
                //--- Check if hold time has expired
                if (_lightHoldActive && _lightHoldTimer.seconds() >= LIGHT_HOLD_TIME)
                {
                    //--- Hold time expired, now turn off lights
                    _lights.setAllOff();
                    _lightHoldActive = false;
                    _lastDetectedTag = -1;
                    _lastProcessedTag = -1;
                }
                else if (!_lightHoldActive && _lastDetectedTag == -1)
                {
                    //--- No tag and no hold active, clear state
                    _lastProcessedTag = -1;
                }

                //--- Scan for tags (only in TELEOP and DEMO modes)
                if (_scanMode != ScanMode.PRE_MATCH)
                {
                    runPitchScan();
                }
            }
            
            //--- Run alignment if enabled (must be inside try block to access _blocks)
            if (_alignmentLockEnabled || _autoAlignForFiring)
            {
                runAlignment();
            }
            
            //--- Override lights when distance is locked
            //--- Shows orange lights to indicate lock: SHORT=1, MEDIUM=2, LONG=3 lights
            if (_distanceLockType == DistanceLockType.SHORT)
            {
                //--- SHORT: left orange only
                _lights.setLeft(Lights.Color.ORANGE);
                _lights.setMiddle(Lights.Color.OFF);
                _lights.setRight(Lights.Color.OFF);
            }
            else if (_distanceLockType == DistanceLockType.MEDIUM)
            {
                //--- MEDIUM: left and center orange
                _lights.setLeft(Lights.Color.ORANGE);
                _lights.setMiddle(Lights.Color.ORANGE);
                _lights.setRight(Lights.Color.OFF);
            }
            else if (_distanceLockType == DistanceLockType.LONG)
            {
                //--- LONG: all three orange
                _lights.setAll(Lights.Color.ORANGE);
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
                _detectedSequence = BallSequence.GPP;
                _kickers.setSequence(Kickers.Sequence.GPP);
                _lights.setLeft(Lights.Color.GREEN);
                _lights.setMiddle(Lights.Color.PURPLE);
                _lights.setRight(Lights.Color.PURPLE);
                break;

            case TAG_SEQUENCE_PGP:
                //--- PGP sequence: set kickers to PGP, solid Purple-Green-Purple
                _detectedSequence = BallSequence.PGP;
                _kickers.setSequence(Kickers.Sequence.PGP);
                _lights.setLeft(Lights.Color.PURPLE);
                _lights.setMiddle(Lights.Color.GREEN);
                _lights.setRight(Lights.Color.PURPLE);
                break;

            case TAG_SEQUENCE_PPG:
                //--- PPG sequence: set kickers to PPG, solid Purple-Purple-Green
                _detectedSequence = BallSequence.PPG;
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
    //--- Call this during init to point camera at obelisk for sequence detection
    //--- Yaw: 0.0 = full right, 0.5 = center, 1.0 = full left
    //--- Pitch: 0.0 = down, 0.5 = level, 1.0 = up
    public void setPreMatchPosition(double yaw, double pitch)
    {
        setScanMode(ScanMode.PRE_MATCH);
        setPosition(yaw, pitch);
        _currentScanPitch = pitch;
        _detectedSequence = BallSequence.UNKNOWN;  // Reset detection
    }

    //--- Get the detected ball sequence (GPP, PGP, PPG, or UNKNOWN)
    public BallSequence getDetectedSequence()
    {
        return _detectedSequence;
    }

    //--- Check if a sequence has been detected
    public boolean isSequenceDetected()
    {
        return _detectedSequence != BallSequence.UNKNOWN;
    }

    //--- Reset detected sequence (call before starting new detection)
    public void resetDetectedSequence()
    {
        _detectedSequence = BallSequence.UNKNOWN;
    }

    //--- Run pre-match detection and update lights to show detected sequence
    //--- Call this in your init loop. Returns the detected sequence.
    //--- Lights will show GPP/PGP/PPG pattern once detected.
    public BallSequence runPreMatchDetection()
    {
        //--- Run camera to detect AprilTags
        run();
        
        //--- If sequence detected, update lights to show the pattern
        if (isSequenceDetected())
        {
            switch (_detectedSequence)
            {
                case GPP:
                    _lights.setLeft(Lights.Color.GREEN);
                    _lights.setMiddle(Lights.Color.PURPLE);
                    _lights.setRight(Lights.Color.PURPLE);
                    break;
                case PGP:
                    _lights.setLeft(Lights.Color.PURPLE);
                    _lights.setMiddle(Lights.Color.GREEN);
                    _lights.setRight(Lights.Color.PURPLE);
                    break;
                case PPG:
                    _lights.setLeft(Lights.Color.PURPLE);
                    _lights.setMiddle(Lights.Color.PURPLE);
                    _lights.setRight(Lights.Color.GREEN);
                    break;
                default:
                    break;
            }
        }
        
        return _detectedSequence;
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
    //--- Applies smoothing based on current smoothing mode (EMA, SMA, or none)
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

        //--- Calculate raw distance using similar triangles
        double rawDistance = (TAG_REAL_SIZE_INCHES * HUSKY_FOCAL_LENGTH) / pixelWidth;
        
        //--- Apply smoothing based on mode
        switch (_smoothingMode)
        {
            case EMA:
                //--- Exponential moving average: smoothed = alpha * new + (1-alpha) * old
                if (_emaDistance < 0)
                {
                    //--- First reading, initialize EMA
                    _emaDistance = rawDistance;
                }
                else
                {
                    _emaDistance = (EMA_ALPHA * rawDistance) + ((1.0 - EMA_ALPHA) * _emaDistance);
                }
                _lastDistanceInches = _emaDistance;
                break;
                
            case SMA:
                //--- Simple moving average: average of last N samples
                _smaBuffer[_smaIndex] = rawDistance;
                _smaIndex = (_smaIndex + 1) % SMA_SAMPLE_COUNT;
                if (_smaCount < SMA_SAMPLE_COUNT) _smaCount++;
                
                //--- Calculate average of valid samples
                double sum = 0;
                for (int i = 0; i < _smaCount; i++)
                {
                    sum += _smaBuffer[i];
                }
                _lastDistanceInches = sum / _smaCount;
                break;
                
            case NONE:
            default:
                //--- No smoothing - use raw reading
                _lastDistanceInches = rawDistance;
                break;
        }
        
        //--- Store this as the last good distance (persists when tag is lost)
        //--- But only if distance is not locked (manual override active)
        if (_distanceLockType == DistanceLockType.NONE)
        {
            _storedDistanceInches = _lastDistanceInches;
        }
    }

    //--- Get the stored distance (last good reading, persists when tag lost)
    public double getStoredDistanceInches()
    {
        return _storedDistanceInches;
    }

    //--- Get stored distance formatted as inches string
    public String getStoredDistanceFormatted()
    {
        if (_storedDistanceInches < 0) return "No reading";
        
        int inches = (int) Math.round(_storedDistanceInches);
        return String.format("%d in", inches);
    }

    //--- Get the velocity adjustment (manual tuning offset)
    public double getVelocityAdjustment()
    {
        return _velocityAdjustment;
    }

    //--- Increase velocity adjustment by increment
    public void increaseVelocityAdjustment()
    {
        _velocityAdjustment = Math.min(500.0, _velocityAdjustment + VELOCITY_INCREMENT);
    }

    //--- Decrease velocity adjustment by increment
    public void decreaseVelocityAdjustment()
    {
        _velocityAdjustment = Math.max(-500.0, _velocityAdjustment - VELOCITY_INCREMENT);
    }

    //--- Reset velocity adjustment to zero
    public void resetVelocityAdjustment()
    {
        _velocityAdjustment = 0.0;
    }

    //--- Set fixed distance for short shots and lock it
    public void setFixedDistanceShort()
    {
        _storedDistanceInches = FIXED_DISTANCE_SHORT;
        _distanceLockType = DistanceLockType.SHORT;
        _isScanning = false;  // Stop scanning since we're using fixed distance
    }

    //--- Set fixed distance for medium shots and lock it
    public void setFixedDistanceMedium()
    {
        _storedDistanceInches = FIXED_DISTANCE_MEDIUM;
        _distanceLockType = DistanceLockType.MEDIUM;
        _isScanning = false;  // Stop scanning since we're using fixed distance
    }

    //--- Set fixed distance for long shots and lock it
    public void setFixedDistanceLong()
    {
        _storedDistanceInches = FIXED_DISTANCE_LONG;
        _distanceLockType = DistanceLockType.LONG;
        _isScanning = false;  // Stop scanning since we're using fixed distance
    }

    //--- Unlock distance (allow camera to update it again)
    public void unlockDistance()
    {
        _distanceLockType = DistanceLockType.NONE;
    }

    //--- Check if distance is locked
    public boolean isDistanceLocked()
    {
        return _distanceLockType != DistanceLockType.NONE;
    }

    //--- Get current distance lock type
    public DistanceLockType getDistanceLockType()
    {
        return _distanceLockType;
    }

    //--- Set distance smoothing mode
    public void setSmoothingMode(SmoothingMode mode)
    {
        _smoothingMode = mode;
        resetSmoothing();  // Reset buffers when changing modes
    }

    //--- Get current smoothing mode
    public SmoothingMode getSmoothingMode()
    {
        return _smoothingMode;
    }

    //--- Cycle through smoothing modes: EMA -> SMA -> NONE -> EMA
    public void cycleSmoothingMode()
    {
        switch (_smoothingMode)
        {
            case EMA:
                setSmoothingMode(SmoothingMode.SMA);
                break;
            case SMA:
                setSmoothingMode(SmoothingMode.NONE);
                break;
            case NONE:
            default:
                setSmoothingMode(SmoothingMode.EMA);
                break;
        }
    }

    //--- Reset smoothing buffers (call when starting fresh or changing modes)
    public void resetSmoothing()
    {
        _emaDistance = -1.0;
        _smaIndex = 0;
        _smaCount = 0;
        for (int i = 0; i < SMA_SAMPLE_COUNT; i++)
        {
            _smaBuffer[i] = 0;
        }
    }

    //--- Get smoothing mode as display string
    public String getSmoothingModeString()
    {
        switch (_smoothingMode)
        {
            case EMA:  return "EMA (α=" + EMA_ALPHA + ")";
            case SMA:  return "SMA (" + SMA_SAMPLE_COUNT + " samples)";
            case NONE: return "OFF (raw)";
            default:   return "Unknown";
        }
    }

    //--- Get suggested flywheel velocity based on stored distance and ball count
    //--- Uses stored distance so velocity is stable even if tag is momentarily lost
    //--- Uses lookup table based on ball count (1, 2, or 3 balls)
    //--- @param ballCount Number of balls to fire (1-3)
    //--- @return Suggested RPM, or -1 if no distance reading
    public double getSuggestedVelocity(int ballCount)
    {
        //--- Use stored distance (persists when tag lost)
        if (_storedDistanceInches < 0) return -1.0;
        
        //--- Convert distance to array index (0-119 for 1-120 inches)
        int index = (int) Math.round(_storedDistanceInches) - 1;
        index = Math.max(0, Math.min(119, index));  // Clamp to valid range
        
        //--- Select appropriate table based on ball count
        double baseRPM;
        if (ballCount <= 1) {
            baseRPM = RPM_TABLE_1_BALL[index];
        } else if (ballCount == 2) {
            baseRPM = RPM_TABLE_2_BALLS[index];
        } else {
            baseRPM = RPM_TABLE_3_BALLS[index];
        }
        
        //--- Apply manual adjustment and clamp to valid range
        double rpm = baseRPM + _velocityAdjustment;
        rpm = Math.max(VELOCITY_FLOOR, Math.min(VELOCITY_CEILING, rpm));
        
        return rpm;
    }
    
    //--- Legacy method - get suggested velocity for 1 ball (backward compatibility)
    public double getSuggestedVelocity()
    {
        return getSuggestedVelocity(1);
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

    //--- Get alignment info string for telemetry (shows error and direction)
    public String getAlignmentInfo()
    {
        if (_blocks.length == 0)
        {
            return "NO BLOCKS";
        }
        int errorX = _blocks[0].x - SCREEN_CENTER_X;
        int absError = Math.abs(errorX);
        if (absError <= ALIGN_DEADBAND)
        {
            return _isSettling ? String.format("SETTLING (%.1fs)", _alignSettleTimer.seconds()) : "IN DEADBAND";
        }
        else
        {
            return String.format("%dpx %s", absError, errorX > 0 ? "→" : "←");
        }
    }

    //--- Check if alignment lock is enabled (manual or auto)
    public boolean isAlignmentActive()
    {
        return _alignmentLockEnabled || _autoAlignForFiring;
    }

    //--- Check if we're looking at a target (blue or red) AND have valid block data
    public boolean isLookingAtTarget()
    {
        return _blocks.length > 0 && 
               (_lastDetectedTag == TAG_BLUE_TARGET || _lastDetectedTag == TAG_RED_TARGET);
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

    //--- Handle targeting controls based on current mode
    //--- gp2 Dpad Up: Switch to AUTO_AIM mode
    //--- gp2 Dpad Down: Switch to MANUAL_TARGET mode
    //--- AUTO_AIM mode: Y=lock on target, A=release lock
    //--- MANUAL_TARGET mode: Y=Close shot, B=Medium shot, A=Long shot
    public void handleTargetingControls()
    {
        //--- Dpad Up - switch to AUTO_AIM mode
        if (_gamepad2.dpad_up)
        {
            if (!_dpadUpPressed)
            {
                _dpadUpPressed = true;
                _targetingMode = TargetingMode.AUTO_AIM;
            }
        }
        else
        {
            _dpadUpPressed = false;
        }

        //--- Dpad Down - switch to MANUAL_TARGET mode
        if (_gamepad2.dpad_down)
        {
            if (!_dpadDownPressed)
            {
                _dpadDownPressed = true;
                _targetingMode = TargetingMode.MANUAL_TARGET;
                //--- Also disable alignment when switching to manual
                disableAlignmentLock();
            }
        }
        else
        {
            _dpadDownPressed = false;
        }

        //--- Handle Y/B/A buttons based on current mode
        if (_targetingMode == TargetingMode.AUTO_AIM)
        {
            handleAutoAimControls();
        }
        else
        {
            handleManualTargetControls();
        }

        //--- Run alignment if enabled (in either mode, alignment can be active)
        if (_alignmentLockEnabled || _autoAlignForFiring)
        {
            runAlignment();
        }
    }

    //--- AUTO_AIM mode controls: Y=lock on, A=release
    private void handleAutoAimControls()
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
    }

    //--- MANUAL_TARGET mode controls: Y=Short, B=Medium, A=Long distance locks
    private void handleManualTargetControls()
    {
        //--- Y button - Short distance (toggle)
        if (_gamepad2.y)
        {
            if (!_yButtonPressed)
            {
                _yButtonPressed = true;
                if (_distanceLockType == DistanceLockType.SHORT)
                {
                    unlockDistance();
                    resetVelocityAdjustment();
                }
                else
                {
                    setFixedDistanceShort();
                }
            }
        }
        else
        {
            _yButtonPressed = false;
        }

        //--- B button - Medium distance (toggle)
        if (_gamepad2.b)
        {
            if (!_bButtonPressed)
            {
                _bButtonPressed = true;
                if (_distanceLockType == DistanceLockType.MEDIUM)
                {
                    unlockDistance();
                    resetVelocityAdjustment();
                }
                else
                {
                    setFixedDistanceMedium();
                }
            }
        }
        else
        {
            _bButtonPressed = false;
        }

        //--- A button - Long distance (toggle)
        if (_gamepad2.a)
        {
            if (!_aButtonPressed)
            {
                _aButtonPressed = true;
                if (_distanceLockType == DistanceLockType.LONG)
                {
                    unlockDistance();
                    resetVelocityAdjustment();
                }
                else
                {
                    setFixedDistanceLong();
                }
            }
        }
        else
        {
            _aButtonPressed = false;
        }
    }

    //--- Legacy method - calls handleTargetingControls for backward compatibility
    public void handleAlignmentControls()
    {
        handleTargetingControls();
    }

    //--- Get the current targeting mode
    public TargetingMode getTargetingMode()
    {
        return _targetingMode;
    }

    //--- Get targeting mode as display string
    public String getTargetingModeString()
    {
        if (_targetingMode == TargetingMode.AUTO_AIM)
        {
            return "AUTO-AIM (Y=Lock, A=Release)";
        }
        else
        {
            return "MANUAL (Y=Short, B=Med, A=Long)";
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
