package org.firstinspires.ftc.teamcode.hardware;

//region --- Imports ---
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
//endregion

public class Kickers
{
    //region --- Constants ---
    //--- Servo Positions (adjust these as needed for each kicker)
    private static final double POSITION_LEFT_DOWN = 0.26;
    private static final double POSITION_LEFT_UP = 0.46;

    private static final double POSITION_MIDDLE_DOWN = 0.25;
    private static final double POSITION_MIDDLE_UP = 0.45;

    private static final double POSITION_RIGHT_DOWN = 0.25;
    private static final double POSITION_RIGHT_UP = 0.45;

    //--- Timing Settings (in seconds)
    private static final double KICK_HOLD_TIME = 0.3;      //--- How long to hold the kick position
    private static final double SEQUENCE_DELAY = 0.5;      //--- Delay between kicks in sequence

    //--- Velocity Presets (RPM) for gamepad1 controls
    private static final double VELOCITY_PRESET_Y = 4000.0;
    private static final double VELOCITY_PRESET_B = 3000.0;
    private static final double VELOCITY_PRESET_X = 2000.0;
    private static final double VELOCITY_PRESET_A = 1500.0;
    //endregion

    //region --- Enums ---
    //--- Ball colors
    public enum BallColor
    {
        PURPLE,
        GREEN
    }

    //--- Firing sequences (order to fire based on ball color pattern)
    public enum Sequence
    {
        GPP,    //--- Green, Purple, Purple
        PPG,    //--- Purple, Purple, Green
        PGP     //--- Purple, Green, Purple
    }
    //endregion

    //region --- Hardware ---
    private final Servo _servoKickerLeft;
    private final Servo _servoKickerMiddle;
    private final Servo _servoKickerRight;
    private final Gamepad _gamepad1;
    private final Gamepad _gamepad2;
    private final Telemetry _telemetry;
    private final boolean _showInfo;
    private final int _robotVersion;

    //--- Flywheel reference for velocity-based firing
    private Flywheel _flywheel = null;

    //--- Camera reference for alignment-based firing
    private Camera _camera = null;
    //endregion

    //region --- State ---
    //--- Ball colors loaded in each kicker position (1=Left, 2=Middle, 3=Right)
    private BallColor _ballColor1 = BallColor.PURPLE;
    private BallColor _ballColor2 = BallColor.PURPLE;
    private BallColor _ballColor3 = BallColor.GREEN;

    //--- Current firing sequence
    private Sequence _sequence = Sequence.GPP;

    //--- Kicker state tracking
    private boolean _kicker1Firing = false;
    private boolean _kicker2Firing = false;
    private boolean _kicker3Firing = false;
    private ElapsedTime _timerKicker1 = new ElapsedTime();
    private ElapsedTime _timerKicker2 = new ElapsedTime();
    private ElapsedTime _timerKicker3 = new ElapsedTime();

    //--- Sequence firing state
    private boolean _sequenceFiring = false;
    private int _sequenceStep = 0;
    private int[] _firingOrder = new int[3];
    private ElapsedTime _timerSequence = new ElapsedTime();
    private boolean _waitingForKickComplete = false;

    //--- Velocity-based firing state
    private boolean _velocityFirePending = false;
    private boolean _velocityFireAll = false;           //--- true = fire all, false = fire sequence
    private double _targetVelocity = 2800.0;            //--- Default to calibrated mid-range (~29-66" range)
    private boolean _waitingForVelocity = false;        //--- Waiting for velocity before firing
    private boolean _sequenceWaitingForVelocity = false; //--- Waiting for velocity recovery during sequence

    //--- Alignment-based firing state
    private boolean _waitingForAlignment = false;       //--- Waiting for alignment before firing
    private boolean _alignmentFireAll = false;          //--- true = fire all, false = fire sequence

    //--- Input debouncing
    private boolean _triggerWasPressed = false;
    private boolean _bumperWasPressed = false;
    private boolean _leftBumperWasPressed = false;
    private boolean _yPressed = false;
    private boolean _bPressed = false;
    private boolean _xPressed = false;
    private boolean _aPressed = false;

    //--- Fine tune test mode state
    private int _tuneMode = 0;  //--- 0=LeftDown, 1=LeftUp, 2=MiddleDown, 3=MiddleUp, 4=RightDown, 5=RightUp
    private double _tuneLeftDown = POSITION_LEFT_DOWN;
    private double _tuneLeftUp = POSITION_LEFT_UP;
    private double _tuneMiddleDown = POSITION_MIDDLE_DOWN;
    private double _tuneMiddleUp = POSITION_MIDDLE_UP;
    private double _tuneRightDown = POSITION_RIGHT_DOWN;
    private double _tuneRightUp = POSITION_RIGHT_UP;
    private boolean _tuneYPressed = false;
    private boolean _tuneAPressed = false;
    private boolean _tuneBPressed = false;
    private boolean _tuneXPressed = false;
    private boolean _tuneInitialized = false;
    //endregion

    //region --- Constructor ---
    public Kickers(
            Servo servoKickerLeft,
            Servo servoKickerMiddle,
            Servo servoKickerRight,
            Flywheel flywheel,
            Gamepad gamepad1,
            Gamepad gamepad2,
            Telemetry telemetry,
            int robotVersion,
            boolean showInfo
    )
    {
        this._servoKickerLeft = servoKickerLeft;
        this._servoKickerMiddle = servoKickerMiddle;
        this._servoKickerRight = servoKickerRight;
        this._flywheel = flywheel;
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
        //--- Set all kickers to down position
        retractAll();
    }
    //endregion

    //region --- Run (call this in your main loop) ---
    public void run()
    {
        //--- Handle individual kicker timing (auto-retract after kick)
        if (_kicker1Firing && _timerKicker1.seconds() >= KICK_HOLD_TIME)
        {
            _servoKickerLeft.setPosition(POSITION_LEFT_DOWN);
            _kicker1Firing = false;
        }

        if (_kicker2Firing && _timerKicker2.seconds() >= KICK_HOLD_TIME)
        {
            _servoKickerMiddle.setPosition(POSITION_MIDDLE_DOWN);
            _kicker2Firing = false;
        }

        if (_kicker3Firing && _timerKicker3.seconds() >= KICK_HOLD_TIME)
        {
            _servoKickerRight.setPosition(POSITION_RIGHT_DOWN);
            _kicker3Firing = false;
        }

        //--- Handle velocity-based firing (waiting for velocity before firing)
        if (_velocityFirePending && _waitingForVelocity)
        {
            if (_flywheel != null && _flywheel.isAtTarget())
            {
                _waitingForVelocity = false;
                
                //--- Now check if we also need alignment
                if (_waitingForAlignment && _camera != null)
                {
                    //--- Still need to wait for alignment
                    return;
                }
                
                if (_velocityFireAll)
                {
                    fireAll();
                }
                else
                {
                    fireSequence();
                }
                _velocityFirePending = false;
            }
        }

        //--- Handle alignment-based firing (waiting for alignment before firing)
        if (_waitingForAlignment && _camera != null)
        {
            if (_camera.isAligned())
            {
                _waitingForAlignment = false;
                
                //--- Check if we're also waiting for velocity
                if (_velocityFirePending && _waitingForVelocity)
                {
                    //--- Still need velocity, will fire when velocity is ready
                    return;
                }
                
                if (_alignmentFireAll)
                {
                    fireAll();
                }
                else
                {
                    fireSequence();
                }
            }
        }

        //--- Handle sequence firing
        if (_sequenceFiring)
        {
            runSequence();
        }
    }

    //--- Handles sequence firing logic
    private void runSequence()
    {
        if (_sequenceStep >= 3)
        {
            //--- Sequence complete
            _sequenceFiring = false;
            _sequenceStep = 0;
            _sequenceWaitingForVelocity = false;
            
            //--- Disable auto-alignment after sequence completes
            if (_camera != null)
            {
                _camera.disableAutoAlignForFiring();
            }
            return;
        }

        //--- If we're waiting for velocity recovery between shots
        if (_sequenceWaitingForVelocity)
        {
            if (_flywheel != null && _flywheel.isAtTarget())
            {
                _sequenceWaitingForVelocity = false;
                //--- Velocity recovered, fire next kicker
                int kickerToFire = _firingOrder[_sequenceStep];
                fireKicker(kickerToFire);
                _waitingForKickComplete = true;
            }
            return;
        }

        if (_waitingForKickComplete)
        {
            //--- Wait for current kick to complete before starting delay
            int currentKicker = _firingOrder[_sequenceStep];
            boolean kickComplete = !isKickerFiring(currentKicker);

            if (kickComplete)
            {
                _waitingForKickComplete = false;
                _timerSequence.reset();
                _sequenceStep++;

                //--- If using velocity-based sequence, wait for velocity recovery
                if (_sequenceStep < 3 && _flywheel != null && _targetVelocity > 0)
                {
                    _sequenceWaitingForVelocity = true;
                }
            }
        }
        else
        {
            //--- Check if delay has passed (or first kick)
            if (_sequenceStep == 0 || _timerSequence.seconds() >= SEQUENCE_DELAY)
            {
                if (_sequenceStep < 3)
                {
                    //--- Fire next kicker in sequence
                    int kickerToFire = _firingOrder[_sequenceStep];
                    fireKicker(kickerToFire);
                    _waitingForKickComplete = true;
                }
            }
        }
    }

    //--- Check if a specific kicker is currently firing
    private boolean isKickerFiring(int kicker)
    {
        switch (kicker)
        {
            case 1: return _kicker1Firing;
            case 2: return _kicker2Firing;
            case 3: return _kicker3Firing;
            default: return false;
        }
    }
    //endregion

    //region --- Control Methods ---

    //--- Call this in your main loop to handle gamepad controls
    //--- Gamepad1 Y/B/X/A: Set velocity presets (4000/3000/2000/1500 RPM)
    //--- Gamepad1 Left Bumper: Auto-set velocity based on camera distance
    //--- Right Trigger: Fire all kickers at once (waits for velocity and alignment if looking at target)
    //--- Right Bumper: Fire in sequence based on ball colors (waits for velocity between shots, aligns if looking at target)
    public void controlKickers()
    {
        //--- Velocity preset buttons (gamepad1)
        handleVelocityPresets();

        //--- Check if we're looking at a target (for auto-alignment)
        boolean lookingAtTarget = (_camera != null && _camera.isLookingAtTarget());

        //--- Right trigger - fire all (detect press, not hold)
        //--- Also cancels any sequence in progress
        if (_gamepad1.right_trigger > 0.1)
        {
            if (!_triggerWasPressed)
            {
                _triggerWasPressed = true;
                _sequenceFiring = false;  //--- Cancel any sequence in progress
                _velocityFirePending = false;  //--- Cancel any pending velocity fire
                _waitingForAlignment = false;  //--- Cancel any pending alignment

                //--- Always use manual _targetVelocity (adjusted with Y/A buttons)

                //--- Start velocity first (so isAtTarget reflects new target)
                if (_flywheel != null && _targetVelocity > 0)
                {
                    _flywheel.setVelocity(_targetVelocity);
                }

                //--- Start alignment if looking at target
                if (lookingAtTarget && _camera != null)
                {
                    _camera.enableAutoAlignForFiring();
                }

                //--- Determine what we need to wait for (AFTER setting velocity)
                boolean needVelocity = (_flywheel != null && _targetVelocity > 0 && !_flywheel.isAtTarget());
                boolean needAlignment = (lookingAtTarget && _camera != null && !_camera.isAligned());

                //--- Check if we can fire immediately or need to wait
                if (!needVelocity && !needAlignment)
                {
                    fireAll();
                    //--- Disable alignment after firing all (no sequence)
                    if (_camera != null)
                    {
                        _camera.disableAutoAlignForFiring();
                    }
                }
                else
                {
                    _velocityFirePending = needVelocity;
                    _velocityFireAll = true;
                    _waitingForVelocity = needVelocity;
                    _waitingForAlignment = needAlignment;
                    _alignmentFireAll = true;
                }
            }
        }
        else
        {
            _triggerWasPressed = false;
        }

        //--- Right bumper - fire in sequence (detect press, not hold)
        //--- Only starts if no sequence is already in progress
        if (_gamepad1.right_bumper)
        {
            if (!_bumperWasPressed && !_sequenceFiring && !_velocityFirePending && !_waitingForAlignment)
            {
                _bumperWasPressed = true;

                //--- Always use manual _targetVelocity (adjusted with Y/A buttons)

                //--- Start velocity first (so isAtTarget reflects new target)
                if (_flywheel != null && _targetVelocity > 0)
                {
                    _flywheel.setVelocity(_targetVelocity);
                }

                //--- Start alignment if looking at target
                if (lookingAtTarget && _camera != null)
                {
                    _camera.enableAutoAlignForFiring();
                }

                //--- Determine what we need to wait for (AFTER setting velocity)
                boolean needVelocity = (_flywheel != null && _targetVelocity > 0 && !_flywheel.isAtTarget());
                boolean needAlignment = (lookingAtTarget && _camera != null && !_camera.isAligned());

                //--- Check if we can fire immediately or need to wait
                if (!needVelocity && !needAlignment)
                {
                    fireSequence();
                }
                else
                {
                    _velocityFirePending = needVelocity;
                    _velocityFireAll = false;
                    _waitingForVelocity = needVelocity;
                    _waitingForAlignment = needAlignment;
                    _alignmentFireAll = false;
                }
            }
        }
        else
        {
            _bumperWasPressed = false;
        }
    }

    //--- Velocity adjustment increment (RPM per button press)
    private static final double VELOCITY_ADJUST_INCREMENT = 50.0;
    private static final double VELOCITY_ADJUST_MIN = 2500.0;  // Absolute floor
    private static final double VELOCITY_ADJUST_MAX = 4000.0;  // Absolute ceiling

    //--- Handle velocity preset buttons on gamepad1
    private void handleVelocityPresets()
    {
        //--- Y button - increase target velocity
        if (_gamepad1.y)
        {
            if (!_yPressed)
            {
                _yPressed = true;
                _targetVelocity = Math.min(VELOCITY_ADJUST_MAX, _targetVelocity + VELOCITY_ADJUST_INCREMENT);
            }
        }
        else
        {
            _yPressed = false;
        }

        //--- A button - decrease target velocity
        if (_gamepad1.a)
        {
            if (!_aPressed)
            {
                _aPressed = true;
                _targetVelocity = Math.max(VELOCITY_ADJUST_MIN, _targetVelocity - VELOCITY_ADJUST_INCREMENT);
            }
        }
        else
        {
            _aPressed = false;
        }

        //--- B button - 3000 RPM
        if (_gamepad1.b)
        {
            if (!_bPressed)
            {
                _bPressed = true;
                _targetVelocity = VELOCITY_PRESET_B;
            }
        }
        else
        {
            _bPressed = false;
        }

        //--- X button - 2000 RPM
        if (_gamepad1.x)
        {
            if (!_xPressed)
            {
                _xPressed = true;
                _targetVelocity = VELOCITY_PRESET_X;
            }
        }
        else
        {
            _xPressed = false;
        }

        //--- Left bumper - auto velocity from camera distance
        if (_gamepad1.left_bumper)
        {
            if (!_leftBumperWasPressed)
            {
                _leftBumperWasPressed = true;
                updateVelocityFromCamera();
            }
        }
        else
        {
            _leftBumperWasPressed = false;
        }
    }

    //--- Update target velocity based on camera distance estimation
    private void updateVelocityFromCamera()
    {
        if (_camera != null)
        {
            double suggested = _camera.getSuggestedVelocity();
            if (suggested > 0)
            {
                _targetVelocity = suggested;
            }
        }
    }

    //--- Set velocity based on camera distance (can be called externally)
    public boolean setVelocityFromCamera()
    {
        if (_camera != null)
        {
            double suggested = _camera.getSuggestedVelocity();
            if (suggested > 0)
            {
                _targetVelocity = suggested;
                return true;
            }
        }
        return false;
    }

    //--- Get the current target velocity
    public double getTargetVelocity()
    {
        return _targetVelocity;
    }

    //endregion

    //region --- Public Methods - Firing ---

    //--- Fire a single kicker (1=Left, 2=Middle, 3=Right)
    public void fireKicker(int kicker)
    {
        switch (kicker)
        {
            case 1:
                _servoKickerLeft.setPosition(POSITION_LEFT_UP);
                _kicker1Firing = true;
                _timerKicker1.reset();
                break;
            case 2:
                _servoKickerMiddle.setPosition(POSITION_MIDDLE_UP);
                _kicker2Firing = true;
                _timerKicker2.reset();
                break;
            case 3:
                _servoKickerRight.setPosition(POSITION_RIGHT_UP);
                _kicker3Firing = true;
                _timerKicker3.reset();
                break;
        }
    }

    //--- Fire all kickers at once
    public void fireAll()
    {
        fireKicker(1);
        fireKicker(2);
        fireKicker(3);
    }

    //--- Fire kickers in sequence based on ball colors and selected sequence
    public void fireSequence()
    {
        //--- Calculate firing order based on sequence and ball colors
        _firingOrder = calculateFiringOrder();
        _sequenceFiring = true;
        _sequenceStep = 0;
        _waitingForKickComplete = false;
    }

    //--- Retract all kickers to down position
    public void retractAll()
    {
        _servoKickerLeft.setPosition(POSITION_LEFT_DOWN);
        _servoKickerMiddle.setPosition(POSITION_MIDDLE_DOWN);
        _servoKickerRight.setPosition(POSITION_RIGHT_DOWN);
        _kicker1Firing = false;
        _kicker2Firing = false;
        _kicker3Firing = false;
    }

    //endregion

    //region --- Public Methods - Ball Color Configuration ---

    //--- Set the ball color for each kicker position
    public void setBallColors(BallColor color1, BallColor color2, BallColor color3)
    {
        _ballColor1 = color1;
        _ballColor2 = color2;
        _ballColor3 = color3;
    }

    //--- Set the ball color for a specific kicker (1=Left, 2=Middle, 3=Right)
    public void setBallColor(int kicker, BallColor color)
    {
        switch (kicker)
        {
            case 1: _ballColor1 = color; break;
            case 2: _ballColor2 = color; break;
            case 3: _ballColor3 = color; break;
        }
    }

    //--- Get the ball color for a specific kicker
    public BallColor getBallColor(int kicker)
    {
        switch (kicker)
        {
            case 1: return _ballColor1;
            case 2: return _ballColor2;
            case 3: return _ballColor3;
            default: return BallColor.PURPLE;
        }
    }

    //endregion

    //region --- Public Methods - Sequence Configuration ---

    //--- Set the firing sequence
    public void setSequence(Sequence sequence)
    {
        _sequence = sequence;
    }

    //--- Get the current firing sequence
    public Sequence getSequence()
    {
        return _sequence;
    }

    //--- Set the camera reference (for alignment-based firing)
    public void setCamera(Camera camera)
    {
        _camera = camera;
    }

    //--- Check if sequence firing is complete
    public boolean isSequenceComplete()
    {
        return !_sequenceFiring && !_velocityFirePending && !_waitingForAlignment;
    }

    //endregion

    //region --- Private Methods ---

    //--- Calculate the firing order based on current sequence and ball colors
    private int[] calculateFiringOrder()
    {
        int[] order = new int[3];
        BallColor[] sequenceColors = getSequenceColors();
        boolean[] used = {false, false, false};

        //--- For each position in the sequence, find a matching ball
        for (int i = 0; i < 3; i++)
        {
            BallColor targetColor = sequenceColors[i];
            order[i] = findKickerWithColor(targetColor, used);
        }

        return order;
    }

    //--- Get the color pattern for the current sequence
    private BallColor[] getSequenceColors()
    {
        switch (_sequence)
        {
            case GPP:
                return new BallColor[] {BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE};
            case PPG:
                return new BallColor[] {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
            case PGP:
                return new BallColor[] {BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE};
            default:
                return new BallColor[] {BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE};
        }
    }

    //--- Find a kicker with the specified ball color that hasn't been used yet
    private int findKickerWithColor(BallColor targetColor, boolean[] used)
    {
        BallColor[] ballColors = {_ballColor1, _ballColor2, _ballColor3};

        for (int i = 0; i < 3; i++)
        {
            if (!used[i] && ballColors[i] == targetColor)
            {
                used[i] = true;
                return i + 1;  //--- Return 1-based kicker number
            }
        }

        //--- Fallback: return first unused kicker
        for (int i = 0; i < 3; i++)
        {
            if (!used[i])
            {
                used[i] = true;
                return i + 1;
            }
        }

        return 1;  //--- Should never reach here
    }

    //endregion

    //region --- Telemetry ---

    //--- Track velocity spin-up time
    private ElapsedTime _velocitySpinUpTimer = new ElapsedTime();
    private double _lastSpinUpTime = 0.0;
    private boolean _wasSpinningUp = false;

    public void getTelemetry()
    {
        //--- Always show target velocity prominently at top (Y/A to adjust)
        double currentRPM = (_flywheel != null) ? _flywheel.getCurrentRPM() : 0;
        _telemetry.addData(">>> TARGET VELOCITY", "%.0f RPM (Y+/A-)", _targetVelocity);
        _telemetry.addData("    Current RPM", "%.0f", currentRPM);
        
        //--- Track spin-up timing
        if (_waitingForVelocity && !_wasSpinningUp)
        {
            //--- Just started spinning up
            _velocitySpinUpTimer.reset();
            _wasSpinningUp = true;
        }
        else if (!_waitingForVelocity && _wasSpinningUp)
        {
            //--- Just finished spinning up
            _lastSpinUpTime = _velocitySpinUpTimer.seconds();
            _wasSpinningUp = false;
        }
        
        //--- Show last spin-up time if we have one
        if (_lastSpinUpTime > 0)
        {
            _telemetry.addData("Last Spin-Up", "%.2f sec", _lastSpinUpTime);
        }
        
        //--- Show status of each firing prerequisite
        boolean lookingAtTarget = (_camera != null && _camera.isLookingAtTarget());
        boolean flywheelAtTarget = (_flywheel != null && _flywheel.isAtTarget());
        boolean isAligned = (_camera != null && _camera.isAligned());
        
        //--- Velocity status
        if (_flywheel == null)
        {
            _telemetry.addData("1. Velocity", "NO FLYWHEEL");
        }
        else if (_waitingForVelocity)
        {
            _telemetry.addData("1. Velocity", "SPINNING UP... %.1fs (%.0f RPM)", 
                    _velocitySpinUpTimer.seconds(), _flywheel.getCurrentRPM());
        }
        else if (flywheelAtTarget)
        {
            _telemetry.addData("1. Velocity", "READY ✓ (%.0f RPM)", _flywheel.getCurrentRPM());
        }
        else
        {
            _telemetry.addData("1. Velocity", "Idle (%.0f RPM)", _flywheel.getCurrentRPM());
        }
        
        //--- Alignment status
        if (_camera == null)
        {
            _telemetry.addData("2. Alignment", "NO CAMERA");
        }
        else if (!lookingAtTarget)
        {
            _telemetry.addData("2. Alignment", "No target visible");
            _telemetry.addData("   Stored Dist", _camera.getStoredDistanceFormatted());
        }
        else if (_waitingForAlignment)
        {
            String alignInfo = _camera.getAlignmentInfo();
            _telemetry.addData("2. Alignment", "ALIGNING... %s", alignInfo);
            _telemetry.addData("   Distance", "%s (stored: %s)", 
                    _camera.getDistanceFormatted(), _camera.getStoredDistanceFormatted());
        }
        else if (isAligned)
        {
            String alignInfo = _camera.getAlignmentInfo();
            _telemetry.addData("2. Alignment", "ALIGNED ✓ %s", alignInfo);
            _telemetry.addData("   Distance", "%s (stored: %s)", 
                    _camera.getDistanceFormatted(), _camera.getStoredDistanceFormatted());
        }
        else
        {
            _telemetry.addData("2. Alignment", "Not aligned");
            _telemetry.addData("   Distance", "%s (stored: %s)", 
                    _camera.getDistanceFormatted(), _camera.getStoredDistanceFormatted());
        }
        
        //--- Firing status
        if (_sequenceFiring)
        {
            _telemetry.addData("3. Firing", "SEQUENCE step %d/3", _sequenceStep + 1);
        }
        else if (_sequenceWaitingForVelocity)
        {
            _telemetry.addData("3. Firing", "SEQUENCE waiting for velocity recovery");
        }
        else if (_velocityFirePending)
        {
            _telemetry.addData("3. Firing", "PENDING (waiting...)");
        }
        else if (_kicker1Firing || _kicker2Firing || _kicker3Firing)
        {
            _telemetry.addData("3. Firing", "FIRED! K1:%s K2:%s K3:%s", 
                    _kicker1Firing ? "▲" : "▼",
                    _kicker2Firing ? "▲" : "▼",
                    _kicker3Firing ? "▲" : "▼");
        }
        else
        {
            _telemetry.addData("3. Firing", "Ready to fire");
        }

        if (_showInfo)
        {
            _telemetry.addData("Kicker 1 (Left)", "Pos: %.2f, Ball: %s", 
                    _servoKickerLeft.getPosition(), _ballColor1);
            _telemetry.addData("Kicker 2 (Middle)", "Pos: %.2f, Ball: %s", 
                    _servoKickerMiddle.getPosition(), _ballColor2);
            _telemetry.addData("Kicker 3 (Right)", "Pos: %.2f, Ball: %s", 
                    _servoKickerRight.getPosition(), _ballColor3);
            _telemetry.addData("Sequence", _sequence);
        }
    }

    //endregion

    //region --- Test Mode ---

    //--- Fine tune servo positions using gamepad2
    //--- Y: Increment position (+0.005)
    //--- A: Decrement position (-0.005)
    //--- B: Next mode (Left Down -> Left Up -> Middle Down -> etc.)
    //--- X: Previous mode
    //--- Dpad Up: Test fire current kicker (move to up position briefly)
    //--- Dpad Down: Retract current kicker (move to down position)
    public void fineTunePositions()
    {
        String[] modeNames = {"Left DOWN", "Left UP", "Middle DOWN", "Middle UP", "Right DOWN", "Right UP"};

        //--- Initialize servo position on first call
        if (!_tuneInitialized)
        {
            _tuneInitialized = true;
            applyTunePosition();
        }

        //--- Y button - increment position
        if (_gamepad2.y)
        {
            if (!_tuneYPressed)
            {
                _tuneYPressed = true;
                adjustTunePosition(0.005);
            }
        }
        else
        {
            _tuneYPressed = false;
        }

        //--- A button - decrement position
        if (_gamepad2.a)
        {
            if (!_tuneAPressed)
            {
                _tuneAPressed = true;
                adjustTunePosition(-0.005);
            }
        }
        else
        {
            _tuneAPressed = false;
        }

        //--- B button - next mode
        if (_gamepad2.b)
        {
            if (!_tuneBPressed)
            {
                _tuneBPressed = true;
                _tuneMode = (_tuneMode + 1) % 6;
                applyTunePosition();
            }
        }
        else
        {
            _tuneBPressed = false;
        }

        //--- X button - previous mode
        if (_gamepad2.x)
        {
            if (!_tuneXPressed)
            {
                _tuneXPressed = true;
                _tuneMode = (_tuneMode - 1 + 6) % 6;
                applyTunePosition();
            }
        }
        else
        {
            _tuneXPressed = false;
        }

        //--- Dpad Up - test fire current kicker (move to UP position)
        if (_gamepad2.dpad_up)
        {
            int currentKicker = (_tuneMode / 2) + 1;  //--- 0,1->1, 2,3->2, 4,5->3
            switch (currentKicker)
            {
                case 1: _servoKickerLeft.setPosition(_tuneLeftUp); break;
                case 2: _servoKickerMiddle.setPosition(_tuneMiddleUp); break;
                case 3: _servoKickerRight.setPosition(_tuneRightUp); break;
            }
        }

        //--- Dpad Down - retract current kicker (move to DOWN position)
        if (_gamepad2.dpad_down)
        {
            int currentKicker = (_tuneMode / 2) + 1;
            switch (currentKicker)
            {
                case 1: _servoKickerLeft.setPosition(_tuneLeftDown); break;
                case 2: _servoKickerMiddle.setPosition(_tuneMiddleDown); break;
                case 3: _servoKickerRight.setPosition(_tuneRightDown); break;
            }
        }

        //--- Display telemetry
        _telemetry.addData("--- KICKER FINE TUNE ---", "");
        _telemetry.addData("Current Mode", ">>> %s <<<", modeNames[_tuneMode]);
        _telemetry.addData("Controls", "Y=+0.005, A=-0.005, B=Next, X=Prev");
        _telemetry.addData("Test", "DpadUp=Fire, DpadDown=Retract");
        _telemetry.addData("------------------------", "");
        _telemetry.addData("Left", "Down: %.3f | Up: %.3f", _tuneLeftDown, _tuneLeftUp);
        _telemetry.addData("Middle", "Down: %.3f | Up: %.3f", _tuneMiddleDown, _tuneMiddleUp);
        _telemetry.addData("Right", "Down: %.3f | Up: %.3f", _tuneRightDown, _tuneRightUp);
        _telemetry.addData("------------------------", "");
    }

    //--- Adjust the current tune position
    private void adjustTunePosition(double delta)
    {
        switch (_tuneMode)
        {
            case 0: _tuneLeftDown = clamp(_tuneLeftDown + delta); break;
            case 1: _tuneLeftUp = clamp(_tuneLeftUp + delta); break;
            case 2: _tuneMiddleDown = clamp(_tuneMiddleDown + delta); break;
            case 3: _tuneMiddleUp = clamp(_tuneMiddleUp + delta); break;
            case 4: _tuneRightDown = clamp(_tuneRightDown + delta); break;
            case 5: _tuneRightUp = clamp(_tuneRightUp + delta); break;
        }
        applyTunePosition();
    }

    //--- Apply the current tune position to the servo
    private void applyTunePosition()
    {
        switch (_tuneMode)
        {
            case 0: _servoKickerLeft.setPosition(_tuneLeftDown); break;
            case 1: _servoKickerLeft.setPosition(_tuneLeftUp); break;
            case 2: _servoKickerMiddle.setPosition(_tuneMiddleDown); break;
            case 3: _servoKickerMiddle.setPosition(_tuneMiddleUp); break;
            case 4: _servoKickerRight.setPosition(_tuneRightDown); break;
            case 5: _servoKickerRight.setPosition(_tuneRightUp); break;
        }
    }

    //--- Clamp value between 0 and 1
    private double clamp(double value)
    {
        return Math.max(0.0, Math.min(1.0, value));
    }

    //endregion
}