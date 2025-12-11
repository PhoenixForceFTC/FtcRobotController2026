package org.firstinspires.ftc.teamcode.hardware;

//region --- Imports ---
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
//endregion

/**
 * Lights - Manages robot LED lights with a priority-based mode system.
 * 
 * Each subsystem updates its "slot" with colors via setXxxSlot() methods.
 * The active mode determines which slot is displayed.
 * Higher priority modes override lower priority modes.
 * 
 * Priority (highest to lowest):
 * - INTAKE:        Ball colors during intake (GREEN/PURPLE/RED/OFF)
 * - KICKSTAND:     Red/Orange/Yellow when kickstand is down
 * - DISTANCE_LOCK: Orange lights showing distance lock level (1/2/3 lights)
 * - CAMERA_TARGET: Blue/Red lights when tracking targets
 * - OFF/DEFAULT:   All lights off (no target = off)
 * 
 * Usage:
 *   // Subsystem updates its slot every loop:
 *   lights.setIntakeSlot(leftColor, middleColor, rightColor);
 *   
 *   // Subsystem activates its mode when needed:
 *   lights.setMode(LightMode.INTAKE);
 *   
 *   // Subsystem releases its mode when done:
 *   lights.releaseMode(LightMode.INTAKE);
 *   
 *   // Or force a specific mode (ignores priority):
 *   lights.forceMode(LightMode.DEFAULT);
 */
public class Lights
{
    //region --- Constants ---
    //--- Blink Speeds (in seconds)
    private static final double BLINK_FAST_INTERVAL = 0.15;
    private static final double BLINK_SLOW_INTERVAL = 0.5;

    //--- Pulse Pattern Settings
    private static final double PULSE_SOLID_DURATION = 2.0;
    //endregion

    //region --- Enums ---
    public enum Color
    {
        OFF(0.0),
        RED(0.29),
        ORANGE(0.33),
        YELLOW(0.38),
        GREEN(0.51),
        BLUE(0.61),
        PURPLE(0.69),
        WHITE(1.0);

        private final double position;

        Color(double position)
        {
            this.position = position;
        }

        public double getPosition()
        {
            return position;
        }
    }

    public enum Blink
    {
        NONE,
        SLOW,
        FAST,
        PULSE
    }

    /**
     * Light modes with priority levels.
     * Higher ordinal = higher priority.
     * When multiple modes are requested, the highest priority wins.
     */
    public enum LightMode
    {
        OFF,            // All lights off (lowest priority, same as no target)
        DEFAULT,        // Same as OFF - no target means lights off
        CAMERA_TARGET,  // Blue/Red based on detected target
        DISTANCE_LOCK,  // Orange pattern (1/2/3 lights)
        KICKSTAND,      // Red/Orange/Yellow when kickstand is down
        INTAKE,         // Ball colors during intake (highest priority for teleop)
        MANUAL          // Direct control - bypasses slot system
    }
    //endregion

    //region --- Hardware ---
    private final Servo _servoLightLeft;
    private final Servo _servoLightMiddle;
    private final Servo _servoLightRight;
    private final Gamepad _gamepad1;
    private final Gamepad _gamepad2;
    private final Telemetry _telemetry;
    private final boolean _showInfo;
    private final int _robotVersion;
    //endregion

    //region --- Mode State ---
    private LightMode _activeMode = LightMode.DEFAULT;
    private boolean _modeEnabled = true;  // When false, slot system is bypassed
    private boolean _kickstandActive = false;  // True when kickstand is DOWN (not just checking slot)

    //--- Slots: each subsystem updates its own slot, run() picks which to display
    private Color[] _defaultSlot = { Color.OFF, Color.OFF, Color.OFF };  // Default = OFF (no target)
    private Color[] _cameraSlot = { Color.OFF, Color.OFF, Color.OFF };
    private Color[] _distanceLockSlot = { Color.OFF, Color.OFF, Color.OFF };
    private Color[] _kickstandSlot = { Color.RED, Color.ORANGE, Color.YELLOW };  // Kickstand shows init pattern
    private Color[] _intakeSlot = { Color.OFF, Color.OFF, Color.OFF };
    //endregion

    //region --- Blink State ---
    private Color _leftColor = Color.OFF;
    private Color _middleColor = Color.OFF;
    private Color _rightColor = Color.OFF;

    private Blink _leftBlink = Blink.NONE;
    private Blink _middleBlink = Blink.NONE;
    private Blink _rightBlink = Blink.NONE;

    //--- Shared timers so all lights blink in sync
    private ElapsedTime _timerSlow = new ElapsedTime();
    private ElapsedTime _timerFast = new ElapsedTime();
    private boolean _slowOn = true;
    private boolean _fastOn = true;

    //--- Pulse pattern state (blink X times, then solid for Y seconds)
    private ElapsedTime _timerPulse = new ElapsedTime();
    private int _pulseBlinkCount = 0;
    private int _pulseBlinkTarget = 4;
    private boolean _pulseOn = true;
    private boolean _pulseInSolidPhase = false;

    //--- Test mode state
    private double _testPosition = 0.0;
    //endregion

    //region --- Constructor ---
    public Lights(
            Servo servoLightLeft,
            Servo servoLightMiddle,
            Servo servoLightRight,
            Gamepad gamepad1,
            Gamepad gamepad2,
            Telemetry telemetry,
            int robotVersion,
            boolean showInfo
    )
    {
        this._servoLightLeft = servoLightLeft;
        this._servoLightMiddle = servoLightMiddle;
        this._servoLightRight = servoLightRight;
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
        //--- Set lights OFF on initialization (pre-match will show pattern separately)
        _activeMode = LightMode.OFF;
        _modeEnabled = true;
        _kickstandActive = false;
        applySlot(_defaultSlot);  // Default slot is now OFF
    }
    //endregion

    //region --- Mode Control ---

    /**
     * Set the active light mode. Higher priority modes override lower priority modes.
     * @param mode The mode to activate
     */
    public void setMode(LightMode mode)
    {
        if (mode.ordinal() >= _activeMode.ordinal())
        {
            _activeMode = mode;
        }
        //--- Track kickstand state (stays active even when overridden by higher priority)
        if (mode == LightMode.KICKSTAND)
        {
            _kickstandActive = true;
        }
    }

    /**
     * Release a mode. If this was the active mode, drops to the next lower priority mode.
     * @param mode The mode to release
     */
    public void releaseMode(LightMode mode)
    {
        //--- If releasing KICKSTAND mode, clear the active flag
        if (mode == LightMode.KICKSTAND)
        {
            _kickstandActive = false;
        }
        
        if (_activeMode == mode)
        {
            //--- Find the highest priority mode that should be active
            //--- Check in priority order: KICKSTAND (if active), DISTANCE_LOCK, CAMERA_TARGET, then OFF
            if (_kickstandActive)
            {
                _activeMode = LightMode.KICKSTAND;
            }
            else if (hasSlotColors(_distanceLockSlot))
            {
                _activeMode = LightMode.DISTANCE_LOCK;
            }
            else if (hasSlotColors(_cameraSlot))
            {
                _activeMode = LightMode.CAMERA_TARGET;
            }
            else
            {
                _activeMode = LightMode.OFF;  // No target = OFF
            }
        }
    }

    /**
     * Force a specific mode, ignoring priority rules.
     * Use this for special cases like auto pre-match display.
     * @param mode The mode to force
     */
    public void forceMode(LightMode mode)
    {
        _activeMode = mode;
    }

    /**
     * Get the current active mode.
     */
    public LightMode getActiveMode()
    {
        return _activeMode;
    }

    /**
     * Enable or disable the mode system.
     * When disabled, direct setLeft/setMiddle/setRight calls work as before.
     * When enabled (default), the slot system manages lights.
     */
    public void setModeEnabled(boolean enabled)
    {
        _modeEnabled = enabled;
    }

    /**
     * Check if a slot has any non-OFF colors set.
     */
    private boolean hasSlotColors(Color[] slot)
    {
        return slot[0] != Color.OFF || slot[1] != Color.OFF || slot[2] != Color.OFF;
    }

    //endregion

    //region --- Slot Updates (subsystems call these) ---

    /**
     * Update the INTAKE slot colors. Call this every loop when intake is running.
     * These colors are displayed when mode is INTAKE.
     */
    public void setIntakeSlot(Color left, Color middle, Color right)
    {
        _intakeSlot[0] = left;
        _intakeSlot[1] = middle;
        _intakeSlot[2] = right;
    }

    /**
     * Update the CAMERA_TARGET slot colors. Call this every loop when camera is tracking.
     * These colors are displayed when mode is CAMERA_TARGET.
     */
    public void setCameraSlot(Color left, Color middle, Color right)
    {
        _cameraSlot[0] = left;
        _cameraSlot[1] = middle;
        _cameraSlot[2] = right;
    }

    /**
     * Update the CAMERA_TARGET slot with a single color for all lights.
     */
    public void setCameraSlot(Color color)
    {
        setCameraSlot(color, color, color);
    }

    /**
     * Update the DISTANCE_LOCK slot colors. Call this when distance is locked.
     * These colors are displayed when mode is DISTANCE_LOCK.
     */
    public void setDistanceLockSlot(Color left, Color middle, Color right)
    {
        _distanceLockSlot[0] = left;
        _distanceLockSlot[1] = middle;
        _distanceLockSlot[2] = right;
    }

    /**
     * Update the DEFAULT slot colors. Normally Red/Orange/Yellow.
     */
    public void setDefaultSlot(Color left, Color middle, Color right)
    {
        _defaultSlot[0] = left;
        _defaultSlot[1] = middle;
        _defaultSlot[2] = right;
    }

    /**
     * Update the KICKSTAND slot colors. Always Red/Orange/Yellow.
     * Called when kickstand goes down.
     */
    public void setKickstandSlot(Color left, Color middle, Color right)
    {
        _kickstandSlot[0] = left;
        _kickstandSlot[1] = middle;
        _kickstandSlot[2] = right;
    }

    /**
     * Clear all slots (set to OFF).
     */
    public void clearAllSlots()
    {
        _intakeSlot[0] = Color.OFF;
        _intakeSlot[1] = Color.OFF;
        _intakeSlot[2] = Color.OFF;
        _cameraSlot[0] = Color.OFF;
        _cameraSlot[1] = Color.OFF;
        _cameraSlot[2] = Color.OFF;
        _distanceLockSlot[0] = Color.OFF;
        _distanceLockSlot[1] = Color.OFF;
        _distanceLockSlot[2] = Color.OFF;
        //--- Note: kickstand slot keeps its pattern (Red/Orange/Yellow)
    }

    //endregion

    //region --- Run (call this in your main loop) ---
    public void run()
    {
        //--- Update shared timers for SLOW blink
        if (_timerSlow.seconds() >= BLINK_SLOW_INTERVAL)
        {
            _slowOn = !_slowOn;
            _timerSlow.reset();
        }

        //--- Update shared timers for FAST blink
        if (_timerFast.seconds() >= BLINK_FAST_INTERVAL)
        {
            _fastOn = !_fastOn;
            _timerFast.reset();
        }

        //--- Update PULSE pattern state
        if (_pulseInSolidPhase)
        {
            //--- In solid phase, wait for duration then restart blinking
            if (_timerPulse.seconds() >= PULSE_SOLID_DURATION)
            {
                _pulseInSolidPhase = false;
                _pulseBlinkCount = 0;
                _pulseOn = true;
                _timerPulse.reset();
            }
        }
        else
        {
            //--- In blink phase
            if (_timerPulse.seconds() >= BLINK_SLOW_INTERVAL)
            {
                _pulseOn = !_pulseOn;
                _timerPulse.reset();

                //--- Count each full blink cycle (on->off counts as 1)
                if (!_pulseOn)
                {
                    _pulseBlinkCount++;
                    if (_pulseBlinkCount >= _pulseBlinkTarget)
                    {
                        //--- Done blinking, switch to solid phase
                        _pulseInSolidPhase = true;
                        _pulseOn = true;
                        _timerPulse.reset();
                    }
                }
            }
        }

        //--- Apply colors from active mode's slot (if mode system is enabled)
        if (_modeEnabled && _activeMode != LightMode.MANUAL)
        {
            Color[] activeSlot;
            switch (_activeMode)
            {
                case INTAKE:
                    activeSlot = _intakeSlot;
                    break;
                case KICKSTAND:
                    activeSlot = _kickstandSlot;
                    break;
                case DISTANCE_LOCK:
                    activeSlot = _distanceLockSlot;
                    break;
                case CAMERA_TARGET:
                    activeSlot = _cameraSlot;
                    break;
                case OFF:
                case DEFAULT:
                default:
                    //--- No target = lights OFF
                    activeSlot = new Color[] { Color.OFF, Color.OFF, Color.OFF };
                    break;
            }
            applySlot(activeSlot);
        }

        //--- Handle blinking (for any lights that have blink set)
        if (_leftBlink != Blink.NONE)
        {
            boolean isOn = getBlinkState(_leftBlink);
            _servoLightLeft.setPosition(isOn ? _leftColor.getPosition() : Color.OFF.getPosition());
        }

        if (_middleBlink != Blink.NONE)
        {
            boolean isOn = getBlinkState(_middleBlink);
            _servoLightMiddle.setPosition(isOn ? _middleColor.getPosition() : Color.OFF.getPosition());
        }

        if (_rightBlink != Blink.NONE)
        {
            boolean isOn = getBlinkState(_rightBlink);
            _servoLightRight.setPosition(isOn ? _rightColor.getPosition() : Color.OFF.getPosition());
        }
    }

    /**
     * Apply a slot's colors to the lights (no blink).
     */
    private void applySlot(Color[] slot)
    {
        _leftColor = slot[0];
        _middleColor = slot[1];
        _rightColor = slot[2];
        _leftBlink = Blink.NONE;
        _middleBlink = Blink.NONE;
        _rightBlink = Blink.NONE;
        _servoLightLeft.setPosition(slot[0].getPosition());
        _servoLightMiddle.setPosition(slot[1].getPosition());
        _servoLightRight.setPosition(slot[2].getPosition());
    }

    //--- Helper to get the current on/off state for a blink mode
    private boolean getBlinkState(Blink blink)
    {
        switch (blink)
        {
            case SLOW:
                return _slowOn;
            case FAST:
                return _fastOn;
            case PULSE:
                return _pulseOn;
            default:
                return true;
        }
    }
    //endregion

    //region --- Public Methods - Set All Lights ---

    //--- Set all lights to the same color enum
    public void setAll(Color color)
    {
        setLeft(color);
        setMiddle(color);
        setRight(color);
    }

    //endregion

    //region --- Public Methods - Set Individual Lights by Color Enum ---

    public void setLeft(Color color)
    {
        setLeft(color, Blink.NONE);
    }

    public void setLeft(Color color, Blink blink)
    {
        _leftColor = color;
        _leftBlink = blink;
        if (blink == Blink.NONE)
        {
            _servoLightLeft.setPosition(color.getPosition());
        }
    }

    public void setLeft(Color color, int pulseCount)
    {
        _leftColor = color;
        _leftBlink = Blink.PULSE;
        setPulseCount(pulseCount);
    }

    public void setMiddle(Color color)
    {
        setMiddle(color, Blink.NONE);
    }

    public void setMiddle(Color color, Blink blink)
    {
        _middleColor = color;
        _middleBlink = blink;
        if (blink == Blink.NONE)
        {
            _servoLightMiddle.setPosition(color.getPosition());
        }
    }

    public void setMiddle(Color color, int pulseCount)
    {
        _middleColor = color;
        _middleBlink = Blink.PULSE;
        setPulseCount(pulseCount);
    }

    public void setRight(Color color)
    {
        setRight(color, Blink.NONE);
    }

    public void setRight(Color color, Blink blink)
    {
        _rightColor = color;
        _rightBlink = blink;
        if (blink == Blink.NONE)
        {
            _servoLightRight.setPosition(color.getPosition());
        }
    }

    public void setRight(Color color, int pulseCount)
    {
        _rightColor = color;
        _rightBlink = Blink.PULSE;
        setPulseCount(pulseCount);
    }

    //endregion

    //region --- Public Methods - Set All Lights with Blink ---

    public void setAll(Color color, Blink blink)
    {
        setLeft(color, blink);
        setMiddle(color, blink);
        setRight(color, blink);
    }

    public void setAll(Color color, int pulseCount)
    {
        setPulseCount(pulseCount);
        _leftColor = color;
        _leftBlink = Blink.PULSE;
        _middleColor = color;
        _middleBlink = Blink.PULSE;
        _rightColor = color;
        _rightBlink = Blink.PULSE;
    }

    //--- Set the pulse count and reset the pulse state
    private void setPulseCount(int count)
    {
        _pulseBlinkTarget = count + 1;
        _pulseBlinkCount = 0;
        _pulseOn = true;
        _pulseInSolidPhase = false;
        _timerPulse.reset();
    }

    //endregion

    //region --- Public Methods - Convenience Color Methods ---

    public void setAllRed() { setAll(Color.RED); }
    public void setAllYellow() { setAll(Color.YELLOW); }
    public void setAllGreen() { setAll(Color.GREEN); }
    public void setAllBlue() { setAll(Color.BLUE); }
    public void setAllPurple() { setAll(Color.PURPLE); }
    public void setAllWhite() { setAll(Color.WHITE); }
    public void setAllOff() { setAll(Color.OFF); }

    //endregion

    //region --- Test Mode ---

    //--- Use gamepad2 dpad up/down to adjust light position value
    //--- Displays current position in telemetry for finding color values
    public void testColors()
    {
        if (_gamepad2.dpad_up)
        {
            _testPosition += 0.01;
            sleep(200);
        }
        else if (_gamepad2.dpad_down)
        {
            _testPosition -= 0.01;
            sleep(200);
        }

        //--- Clamp value between 0 and 1
        _testPosition = Range.clip(_testPosition, 0.0, 1.0);

        //--- Set all lights to test position
        _servoLightLeft.setPosition(_testPosition);
        _servoLightMiddle.setPosition(_testPosition);
        _servoLightRight.setPosition(_testPosition);

        _telemetry.addData("Light Test", "%4.2f", _testPosition);
    }

    //--- Use gamepad2 Y/B/A/X buttons to test different light patterns
    //--- Y: Green Purple Green (solid)
    //--- B: Purple Green Purple (slow blink)
    //--- A: Blue Red Yellow (fast blink)
    //--- X: Blue (solid) Red (solid) Yellow (fast blink)
    public void testPattern()
    {
        if (_gamepad2.y)
        {
            //--- Pattern 1: Green Purple Green (solid)
            setLeft(Color.GREEN);
            setMiddle(Color.PURPLE);
            setRight(Color.GREEN);
            _telemetry.addData("Light Pattern", "Y: Green Purple Green (solid)");
        }
        else if (_gamepad2.b)
        {
            //--- Pattern 2: Purple Green Purple (slow blink)
            setLeft(Color.PURPLE, Blink.SLOW);
            setMiddle(Color.GREEN, Blink.SLOW);
            setRight(Color.PURPLE, Blink.SLOW);
            _telemetry.addData("Light Pattern", "B: Purple Green Purple (slow)");
        }
        else if (_gamepad2.a)
        {
            //--- Pattern 3: Blue Red Yellow (fast blink)
            setLeft(Color.BLUE, Blink.FAST);
            setMiddle(Color.RED, Blink.FAST);
            setRight(Color.YELLOW, Blink.FAST);
            _telemetry.addData("Light Pattern", "A: Blue Red Yellow (fast)");
        }
        else if (_gamepad2.x)
        {
            //--- Pattern 4: Blue (solid) Red (solid) Yellow (fast blink)
            setLeft(Color.BLUE);
            setMiddle(Color.RED);
            setRight(Color.YELLOW, 3);
            _telemetry.addData("Light Pattern", "X: Blue Red Yellow(fast)");
        }
    }

    private void sleep(long milliseconds)
    {
        try { Thread.sleep(milliseconds); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }

    //endregion

    //region --- Telemetry ---

    public void getTelemetry()
    {
        if (_showInfo)
        {
            _telemetry.addData("Light Left", "%4.2f", _servoLightLeft.getPosition());
            _telemetry.addData("Light Middle", "%4.2f", _servoLightMiddle.getPosition());
            _telemetry.addData("Light Right", "%4.2f", _servoLightRight.getPosition());
        }
    }

    //endregion
}
