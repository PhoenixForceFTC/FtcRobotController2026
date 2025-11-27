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
    private int _robotVersion;
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

    //--- Input debouncing
    private boolean _triggerWasPressed = false;
    private boolean _bumperWasPressed = false;

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
    //endregion

    //region --- Constructor ---
    public Kickers(
            Servo servoKickerLeft,
            Servo servoKickerMiddle,
            Servo servoKickerRight,
            Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, int robotVersion, boolean showInfo
    )
    {
        this._servoKickerLeft = servoKickerLeft;
        this._servoKickerMiddle = servoKickerMiddle;
        this._servoKickerRight = servoKickerRight;
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
    //--- Right Trigger: Fire all kickers at once
    //--- Right Bumper: Fire in sequence based on ball colors
    public void controlKickers()
    {
        //--- Right trigger - fire all (detect press, not hold)
        if (_gamepad1.right_trigger > 0.1)
        {
            if (!_triggerWasPressed)
            {
                _triggerWasPressed = true;
                fireAll();
            }
        }
        else
        {
            _triggerWasPressed = false;
        }

        //--- Right bumper - fire in sequence (detect press, not hold)
        if (_gamepad1.right_bumper)
        {
            if (!_bumperWasPressed)
            {
                _bumperWasPressed = true;
                fireSequence();
            }
        }
        else
        {
            _bumperWasPressed = false;
        }
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

    public void getTelemetry()
    {
        if (_showInfo)
        {
            _telemetry.addData("Kicker 1 (Left)", "Pos: %.2f, Ball: %s", 
                    _servoKickerLeft.getPosition(), _ballColor1);
            _telemetry.addData("Kicker 2 (Middle)", "Pos: %.2f, Ball: %s", 
                    _servoKickerMiddle.getPosition(), _ballColor2);
            _telemetry.addData("Kicker 3 (Right)", "Pos: %.2f, Ball: %s", 
                    _servoKickerRight.getPosition(), _ballColor3);
            _telemetry.addData("Sequence", _sequence);
            _telemetry.addData("Sequence Firing", _sequenceFiring);
        }
    }

    //endregion

    //region --- Test Mode ---

    //--- Fine tune servo positions using gamepad2
    //--- Y: Increment position (+0.01)
    //--- A: Decrement position (-0.01)
    //--- B: Next mode (Left Down -> Left Up -> Middle Down -> etc.)
    //--- X: Previous mode
    public void fineTunePositions()
    {
        String[] modeNames = {"Left DOWN", "Left UP", "Middle DOWN", "Middle UP", "Right DOWN", "Right UP"};

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

        //--- Display telemetry
        _telemetry.addData("--- KICKER FINE TUNE ---", "");
        _telemetry.addData("Current Mode", ">>> %s <<<", modeNames[_tuneMode]);
        _telemetry.addData("Controls", "Y=+0.01, A=-0.01, B=Next, X=Prev");
        _telemetry.addData("------------------------", "");
        _telemetry.addData("Left", "Down: %.2f | Up: %.2f", _tuneLeftDown, _tuneLeftUp);
        _telemetry.addData("Middle", "Down: %.2f | Up: %.2f", _tuneMiddleDown, _tuneMiddleUp);
        _telemetry.addData("Right", "Down: %.2f | Up: %.2f", _tuneRightDown, _tuneRightUp);
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