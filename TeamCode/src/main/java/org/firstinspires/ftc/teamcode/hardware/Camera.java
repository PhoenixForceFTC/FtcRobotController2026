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
    private static final double PITCH_CENTER = 0.6;

    //--- Fine tune increment
    private static final double TUNE_INCREMENT = 0.01;

    //--- AprilTag IDs
    private static final int TAG_BLUE_TARGET = 3;
    private static final int TAG_SEQUENCE_GPP = 1;
    private static final int TAG_SEQUENCE_PGP = 5;
    private static final int TAG_SEQUENCE_PPG = 4;
    private static final int TAG_RED_TARGET = 2;

    //--- HuskyLens screen dimensions (for alignment)
    private static final int SCREEN_CENTER_X = 160;  // 320 / 2
    private static final int ALIGN_DEADBAND = 35;    // Pixels from center to consider "aligned" (increased)
    private static final int ALIGN_SLOWZONE = 60;    // Pixels from center to start slowing down
    private static final double ALIGN_SPEED_MIN = 0.08;  // Minimum rotation speed (reduced)
    private static final double ALIGN_SPEED_MAX = 0.25;  // Maximum rotation speed (reduced)
    private static final double ALIGN_SETTLE_TIME = 0.15; // Seconds to wait after reaching deadband
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
    private int _robotVersion;
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
    private int _lastErrorDirection = 0;            // Track direction to detect oscillation
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

            //--- Process detected AprilTags
            if (_blocks.length > 0)
            {
                //--- Get the first detected block (could prioritize by size/distance)
                int detectedId = _blocks[0].id;
                _lastDetectedTag = detectedId;

                //--- Only process if it's a new tag (prevent repeated triggers)
                if (detectedId != _lastProcessedTag)
                {
                    _lastProcessedTag = detectedId;
                    processAprilTag(detectedId);
                }
            }
            else
            {
                //--- No tags detected
                if (_lastDetectedTag != -1)
                {
                    //--- Just lost sight of tag, turn off lights
                    _lights.setAllOff();
                }
                _lastDetectedTag = -1;
                _lastProcessedTag = -1;
            }
        }
        catch (Exception e)
        {
            //--- HuskyLens communication error
            _isConnected = false;
        }
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
                _drive.stopMotors();  // Stop and let brake mode hold
            }
            
            //--- Check if we've been settled long enough
            if (_alignSettleTimer.seconds() >= ALIGN_SETTLE_TIME)
            {
                _isAligned = true;
            }
            
            //--- Keep motors stopped while settling
            _drive.stopMotors();
        }
        //--- In the slow zone - slow approach, very gentle corrections
        else if (absError <= ALIGN_SLOWZONE)
        {
            _isAligned = false;
            _isSettling = false;
            
            //--- Use minimum speed for fine adjustments
            double speed = ALIGN_SPEED_MIN;

            //--- Rotate in the correct direction
            if (errorX > 0)
            {
                _drive.rotateRight(speed);
            }
            else
            {
                _drive.rotateLeft(speed);
            }
        }
        else
        {
            _isAligned = false;
            _isSettling = false;

            //--- Calculate proportional speed based on error
            //--- Faster when far, slower when close
            double proportion = (double)absError / 160.0;
            double speed = ALIGN_SPEED_MIN + (proportion * (ALIGN_SPEED_MAX - ALIGN_SPEED_MIN));
            speed = Math.min(speed, ALIGN_SPEED_MAX);

            //--- Rotate in the correct direction
            if (errorX > 0)
            {
                _drive.rotateRight(speed);
            }
            else
            {
                _drive.rotateLeft(speed);
            }
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
            _telemetry.addData("Last Tag", _lastDetectedTag);
            _telemetry.addData("Blocks Detected", _blocks.length);
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
