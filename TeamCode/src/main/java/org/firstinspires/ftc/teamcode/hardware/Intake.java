package org.firstinspires.ftc.teamcode.hardware;

//region --- Imports ---
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//endregion

public class Intake
{
    //region --- Constants ---
    //--- Intake/Outtake Speed Settings
    private static final double INTAKE_SPEED = 0.75;
    private static final double OUTTAKE_SPEED = 0.33;

    //--- Outtake Duration (in seconds)
    private static final double OUTTAKE_DURATION = 2.0;
    
    //--- Settle delay before outtake (let balls settle after 3 detected)
    private static final double SETTLE_DELAY_SECONDS = 0.5;
    
    //--- Consecutive detection cycles required before triggering auto-stop
    //--- Prevents false triggers when balls roll across sensors
    private static final int CONSECUTIVE_FULL_DETECTIONS_REQUIRED = 10;

    //--- Ball Detection Settings (per-sensor distance thresholds)
    //--- Each sensor may have different mounting distances
    //--- Ball detected when distance < threshold (ball is closer than empty)
    //--- Thresholds set ~2-3mm BELOW measured empty min values
    //--- Left empty: min 77.7, max 82.8 → threshold 75mm (ball must read < 75)
    //--- Center empty: min 90.8, max 100.4 → threshold 88mm (ball must read < 88)
    //--- Right empty: min 92.8, max 98.9 → threshold 90mm (ball must read < 90)
    private static final double LEFT_DISTANCE_THRESHOLD_MM = 75.0;
    private static final double CENTER_DISTANCE_THRESHOLD_MM = 88.0;
    private static final double RIGHT_DISTANCE_THRESHOLD_MM = 90.0;

    //--- Averaging settings
    private static final int AVERAGING_SAMPLES = 5;  // Number of samples to average
    private static final int MINMAX_BUFFER_SIZE = 100;  // Rolling buffer for min/max tracking

    //--- Color Detection Thresholds (REV Color Sensor V3)
    //--- Using RGB ratios for reliable detection
    //--- Background (no ball) has G/R ~1.4, so we need higher thresholds
    //--- Green balls: G/R > 2.5 (your readings show 3.0+)
    //--- Purple balls: B/G > 0.85 AND G/R < 1.8 (distinguishes from green and background)
    private static final double GREEN_RATIO_THRESHOLD = 2.5;   // G/R must exceed this for green
    private static final double PURPLE_BG_THRESHOLD = 0.85;    // B/G must exceed this for purple
    private static final double PURPLE_GR_MAX = 1.8;           // G/R must be below this for purple

    //--- Distance-based unknown detection toggle
    //--- When false, only color ratios are used (no UNKNOWN state from distance check)
    //--- When true, balls detected by distance but not matching color thresholds show as UNKNOWN
    private static final boolean USE_DISTANCE_FOR_UNKNOWNS = false;
    //endregion

    //region --- Enums ---
    public enum BallColor
    {
        NONE,
        GREEN,
        PURPLE,
        UNKNOWN  // Ball detected but color unclear
    }
    //endregion

    //region --- Hardware ---
    private final DcMotor _motorIntake;
    private final Gamepad _gamepad1;
    private final Gamepad _gamepad2;
    private final Telemetry _telemetry;
    private final boolean _showInfo;
    private final int _robotVersion;

    //--- Color Sensors (cast to DistanceSensor for distance reading)
    private ColorSensor _colorSensorLeft = null;
    private ColorSensor _colorSensorCenter = null;
    private ColorSensor _colorSensorRight = null;  // Future use
    private DistanceSensor _distanceSensorLeft = null;
    private DistanceSensor _distanceSensorCenter = null;
    private DistanceSensor _distanceSensorRight = null;  // Future use

    //--- Lights reference for ball indication
    private Lights _lights = null;
    
    //--- Kickstand reference (to check if lights are being used for kickstand)
    private Kickstand _kickstand = null;
    //endregion

    //region --- State ---
    private boolean _intakeOn = false;
    private boolean _triggerWasPressed = false;
    private boolean _outtakeActive = false;
    private boolean _settleActive = false;  // Waiting for balls to settle before outtake
    private ElapsedTime _outtakeTimer = new ElapsedTime();
    private ElapsedTime _settleTimer = new ElapsedTime();
    private int _consecutiveFullDetections = 0;  // Counter for stable 3-ball detection

    //--- Ball detection state
    private BallColor _leftBallColor = BallColor.NONE;
    private BallColor _centerBallColor = BallColor.NONE;
    private BallColor _rightBallColor = BallColor.NONE;

    //--- Sensor averaging buffers (circular buffers for last N readings)
    private double[] _leftDistBuffer = new double[AVERAGING_SAMPLES];
    private int[] _leftRedBuffer = new int[AVERAGING_SAMPLES];
    private int[] _leftGreenBuffer = new int[AVERAGING_SAMPLES];
    private int[] _leftBlueBuffer = new int[AVERAGING_SAMPLES];
    private int _leftBufferIndex = 0;

    private double[] _centerDistBuffer = new double[AVERAGING_SAMPLES];
    private int[] _centerRedBuffer = new int[AVERAGING_SAMPLES];
    private int[] _centerGreenBuffer = new int[AVERAGING_SAMPLES];
    private int[] _centerBlueBuffer = new int[AVERAGING_SAMPLES];
    private int _centerBufferIndex = 0;

    private double[] _rightDistBuffer = new double[AVERAGING_SAMPLES];
    private int[] _rightRedBuffer = new int[AVERAGING_SAMPLES];
    private int[] _rightGreenBuffer = new int[AVERAGING_SAMPLES];
    private int[] _rightBlueBuffer = new int[AVERAGING_SAMPLES];
    private int _rightBufferIndex = 0;

    //--- Averaged values (for telemetry display)
    private double _leftAvgDist, _leftAvgR, _leftAvgG, _leftAvgB;
    private double _centerAvgDist, _centerAvgR, _centerAvgG, _centerAvgB;
    private double _rightAvgDist, _rightAvgR, _rightAvgG, _rightAvgB;

    //--- Min/Max distance tracking (rolling buffer for last 100 readings)
    private double[] _leftMinMaxBuffer = new double[MINMAX_BUFFER_SIZE];
    private double[] _centerMinMaxBuffer = new double[MINMAX_BUFFER_SIZE];
    private double[] _rightMinMaxBuffer = new double[MINMAX_BUFFER_SIZE];
    private int _leftMinMaxIndex = 0, _centerMinMaxIndex = 0, _rightMinMaxIndex = 0;
    private int _leftMinMaxCount = 0, _centerMinMaxCount = 0, _rightMinMaxCount = 0;

    //--- Sticky color flags (for telemetry)
    private boolean _leftIsSticky = false;
    private boolean _centerIsSticky = false;
    private boolean _rightIsSticky = false;
    //endregion

    //region --- Constructor ---
    public Intake(
            DcMotor motorIntake,
            Gamepad gamepad1,
            Gamepad gamepad2,
            Telemetry telemetry,
            int robotVersion,
            boolean showInfo
    )
    {
        this._motorIntake = motorIntake;
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
        _intakeOn = false;
        _outtakeActive = false;
        _settleActive = false;
        stop();
        
        //--- Initialize distance buffers with high values to prevent false ball detection
        resetDistanceBuffers();
    }

    //--- Set color sensors (call after construction)
    public void setColorSensors(ColorSensor left, ColorSensor center, ColorSensor right)
    {
        _colorSensorLeft = left;
        _colorSensorCenter = center;
        _colorSensorRight = right;

        //--- REV Color Sensor V3 also implements DistanceSensor
        if (left != null) _distanceSensorLeft = (DistanceSensor) left;
        if (center != null) _distanceSensorCenter = (DistanceSensor) center;
        if (right != null) _distanceSensorRight = (DistanceSensor) right;
    }

    //--- Set lights reference (call after construction)
    public void setLights(Lights lights)
    {
        _lights = lights;
    }
    
    //--- Set kickstand reference (to check if kickstand is using lights)
    public void setKickstand(Kickstand kickstand)
    {
        _kickstand = kickstand;
    }
    //endregion

    //region --- Run (call this in your main loop) ---
    public void run()
    {
        //--- Only detect balls and update lights while intaking or outtaking
        if (isIntakeOn() || isOuttakeActive() || _settleActive)
        {
            detectBalls();
            updateLights();
            
            //--- Handle settle delay (keep intake running, then outtake)
            if (_settleActive)
            {
                if (_settleTimer.seconds() >= SETTLE_DELAY_SECONDS)
                {
                    //--- Settle complete, now outtake
                    _settleActive = false;
                    _outtakeActive = true;
                    _intakeOn = false;
                    _outtakeTimer.reset();
                    outtake();
                }
                //--- Still settling, keep intake running
            }
            //--- Auto-stop: when 3 balls detected consistently, start settle delay
            //--- Require multiple consecutive cycles to prevent false triggers from rolling balls
            else if (_intakeOn && !_outtakeActive)
            {
                if (getBallCount() >= 3)
                {
                    _consecutiveFullDetections++;
                    if (_consecutiveFullDetections >= CONSECUTIVE_FULL_DETECTIONS_REQUIRED)
                    {
                        _settleActive = true;
                        _settleTimer.reset();
                        _consecutiveFullDetections = 0;
                        //--- Keep intake running during settle
                    }
                }
                else
                {
                    //--- Reset counter if we don't have 3 balls
                    _consecutiveFullDetections = 0;
                }
            }
        }
    }
    //endregion

    //region --- Ball Detection ---

    private void detectBalls()
    {
        //--- Update sensor buffers and detect balls using averaged values
        //--- Apply "sticky" color logic: only upgrade from UNKNOWN to a color, never downgrade
        _leftBallColor = updateAndDetectSticky(
            _colorSensorLeft, _distanceSensorLeft,
            _leftDistBuffer, _leftRedBuffer, _leftGreenBuffer, _leftBlueBuffer,
            _leftBufferIndex, LEFT_DISTANCE_THRESHOLD_MM, "left", _leftBallColor);
        _leftBufferIndex = (_leftBufferIndex + 1) % AVERAGING_SAMPLES;

        _centerBallColor = updateAndDetectSticky(
            _colorSensorCenter, _distanceSensorCenter,
            _centerDistBuffer, _centerRedBuffer, _centerGreenBuffer, _centerBlueBuffer,
            _centerBufferIndex, CENTER_DISTANCE_THRESHOLD_MM, "center", _centerBallColor);
        _centerBufferIndex = (_centerBufferIndex + 1) % AVERAGING_SAMPLES;

        _rightBallColor = updateAndDetectSticky(
            _colorSensorRight, _distanceSensorRight,
            _rightDistBuffer, _rightRedBuffer, _rightGreenBuffer, _rightBlueBuffer,
            _rightBufferIndex, RIGHT_DISTANCE_THRESHOLD_MM, "right", _rightBallColor);
        _rightBufferIndex = (_rightBufferIndex + 1) % AVERAGING_SAMPLES;
    }

    private BallColor updateAndDetectSticky(
        ColorSensor colorSensor, DistanceSensor distanceSensor,
        double[] distBuffer, int[] redBuffer, int[] greenBuffer, int[] blueBuffer,
        int bufferIndex, double distanceThreshold, String sensorName, BallColor previousColor)
    {
        if (colorSensor == null || distanceSensor == null)
        {
            return BallColor.NONE;
        }

        //--- Read current sensor values
        double distance = distanceSensor.getDistance(DistanceUnit.MM);
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        //--- Store in circular buffer
        distBuffer[bufferIndex] = Double.isNaN(distance) ? 999.0 : distance;
        redBuffer[bufferIndex] = red;
        greenBuffer[bufferIndex] = green;
        blueBuffer[bufferIndex] = blue;

        //--- Calculate averages
        double avgDist = 0, avgR = 0, avgG = 0, avgB = 0;
        for (int i = 0; i < AVERAGING_SAMPLES; i++)
        {
            avgDist += distBuffer[i];
            avgR += redBuffer[i];
            avgG += greenBuffer[i];
            avgB += blueBuffer[i];
        }
        avgDist /= AVERAGING_SAMPLES;
        avgR /= AVERAGING_SAMPLES;
        avgG /= AVERAGING_SAMPLES;
        avgB /= AVERAGING_SAMPLES;

        //--- Store averages for telemetry display
        if (sensorName.equals("left"))
        {
            _leftAvgDist = avgDist; _leftAvgR = avgR; _leftAvgG = avgG; _leftAvgB = avgB;
        }
        else if (sensorName.equals("center"))
        {
            _centerAvgDist = avgDist; _centerAvgR = avgR; _centerAvgG = avgG; _centerAvgB = avgB;
        }
        else if (sensorName.equals("right"))
        {
            _rightAvgDist = avgDist; _rightAvgR = avgR; _rightAvgG = avgG; _rightAvgB = avgB;
        }

        //--- Check distance threshold (per-sensor)
        //--- If no ball detected, reset color to NONE
        if (avgDist > distanceThreshold)
        {
            setStickyFlag(sensorName, false);
            return BallColor.NONE;
        }

        //--- Ball is present (under distance threshold), determine color
        //--- Calculate ratios from averaged values (with safety for divide by zero)
        double gToR = (avgR > 0) ? avgG / avgR : 0;
        double bToG = (avgG > 0) ? avgB / avgG : 0;

        //--- Green ball: G/R > 2.5 (background is ~1.4, green balls are 3.0+)
        if (gToR > GREEN_RATIO_THRESHOLD)
        {
            setStickyFlag(sensorName, false);
            return BallColor.GREEN;
        }

        //--- Purple ball: B/G > 0.85 AND G/R < 1.8
        if (bToG > PURPLE_BG_THRESHOLD && gToR < PURPLE_GR_MAX)
        {
            setStickyFlag(sensorName, false);
            return BallColor.PURPLE;
        }

        //--- Ball detected but color unclear
        //--- "Sticky" logic: keep previous color if it was GREEN or PURPLE
        //--- Only show UNKNOWN (red light) if we never got a good reading
        if (previousColor == BallColor.GREEN || previousColor == BallColor.PURPLE)
        {
            setStickyFlag(sensorName, true);
            return previousColor;  // Keep the last good color
        }

        //--- No previous good color
        //--- If using distance for unknowns, show UNKNOWN (red light)
        //--- Otherwise, treat as no ball (NONE) - only trust color detection
        setStickyFlag(sensorName, false);
        return USE_DISTANCE_FOR_UNKNOWNS ? BallColor.UNKNOWN : BallColor.NONE;
    }

    private void setStickyFlag(String sensorName, boolean isSticky)
    {
        if (sensorName.equals("left")) _leftIsSticky = isSticky;
        else if (sensorName.equals("center")) _centerIsSticky = isSticky;
        else if (sensorName.equals("right")) _rightIsSticky = isSticky;
    }

    //--- Calculate min and max from a rolling buffer
    //--- Returns [min, max] array
    private double[] getMinMax(double[] buffer, int count)
    {
        if (count == 0)
        {
            return new double[] { 0, 0 };
        }
        
        double min = Double.MAX_VALUE;
        double max = 0;
        
        for (int i = 0; i < count; i++)
        {
            if (buffer[i] < min) min = buffer[i];
            if (buffer[i] > max) max = buffer[i];
        }
        
        return new double[] { min, max };
    }

    //--- Calculate hue (0-360) from RGB values
    private double calculateHue(int r, int g, int b)
    {
        double red = r / 255.0;
        double green = g / 255.0;
        double blue = b / 255.0;

        double max = Math.max(red, Math.max(green, blue));
        double min = Math.min(red, Math.min(green, blue));
        double delta = max - min;

        if (delta == 0)
        {
            return 0;  // Achromatic (gray)
        }

        double hue;
        if (max == red)
        {
            hue = 60 * (((green - blue) / delta) % 6);
        }
        else if (max == green)
        {
            hue = 60 * (((blue - red) / delta) + 2);
        }
        else
        {
            hue = 60 * (((red - green) / delta) + 4);
        }

        if (hue < 0)
        {
            hue += 360;
        }

        return hue;
    }

    //endregion

    //region --- Light Updates ---

    private void updateLights()
    {
        if (_lights == null)
        {
            return;
        }
        
        //--- Don't update lights if kickstand is down (kickstand uses lights)
        if (_kickstand != null && _kickstand.getTargetPosition() == Kickstand.Position.DOWN)
        {
            return;
        }

        //--- Update left light based on left ball color
        _lights.setLeft(ballColorToLightColor(_leftBallColor));

        //--- Update middle light based on center ball color
        _lights.setMiddle(ballColorToLightColor(_centerBallColor));

        //--- Update right light based on right ball color
        _lights.setRight(ballColorToLightColor(_rightBallColor));
    }

    private Lights.Color ballColorToLightColor(BallColor ballColor)
    {
        switch (ballColor)
        {
            case GREEN:
                return Lights.Color.GREEN;
            case PURPLE:
                return Lights.Color.PURPLE;
            case UNKNOWN:
                return Lights.Color.RED;
            case NONE:
            default:
                return Lights.Color.OFF;
        }
    }

    //--- Restore lights to default state (call when intake stops)
    //--- Sets lights OFF so camera can control them based on detected targets
    public void restoreLights()
    {
        if (_lights != null)
        {
            _lights.setAllOff();  // Turn off so camera can show target colors
        }
    }

    //endregion

    //region --- Control Methods ---

    //--- Call this in your main loop to handle intake controls
    //--- Left Trigger: Toggle intake on/off
    //--- Left Bumper: Outtake for 2 seconds then stop
    public void controlIntake()
    {
        //--- Handle outtake timer
        if (_outtakeActive)
        {
            if (_outtakeTimer.seconds() >= OUTTAKE_DURATION)
            {
                //--- Outtake duration complete - stop
                _outtakeActive = false;
                _intakeOn = false;
                stop();
                restoreLights();
            }
            //--- Still in outtake mode, don't process other inputs
        }
        else
        {
            //--- Handle left bumper - start outtake
            if (_gamepad1.left_bumper)
            {
                _outtakeActive = true;
                _intakeOn = false;
                _outtakeTimer.reset();
                outtake();
            }
            //--- Handle left trigger toggle (detect press, not hold)
            else if (_gamepad1.left_trigger > 0.1)
            {
                if (!_triggerWasPressed)
                {
                    //--- Trigger just pressed - toggle intake
                    _triggerWasPressed = true;
                    _intakeOn = !_intakeOn;

                    if (_intakeOn)
                    {
                        intake();
                    }
                    else
                    {
                        stop();
                        restoreLights();
                    }
                }
            }
            else
            {
                //--- Trigger released
                _triggerWasPressed = false;
            }
        }
    }

    //endregion

    //region --- Public Methods ---

    //--- Reset distance buffers to prevent false ball detection
    private void resetDistanceBuffers()
    {
        for (int i = 0; i < AVERAGING_SAMPLES; i++)
        {
            _leftDistBuffer[i] = 999.0;
            _centerDistBuffer[i] = 999.0;
            _rightDistBuffer[i] = 999.0;
        }
        _leftBufferIndex = 0;
        _centerBufferIndex = 0;
        _rightBufferIndex = 0;
        _consecutiveFullDetections = 0;  // Reset 3-ball detection counter
    }

    //--- Run intake at configured speed
    public void intake()
    {
        //--- Reset buffers to prevent false ball detection from stale data
        resetDistanceBuffers();
        _intakeOn = true;
        _motorIntake.setPower(INTAKE_SPEED);
    }

    //--- Run intake at custom speed
    public void intake(double power)
    {
        //--- Reset buffers to prevent false ball detection from stale data
        resetDistanceBuffers();
        _intakeOn = true;
        _motorIntake.setPower(Math.abs(power));
    }

    //--- Run outtake (reverse) at configured speed
    public void outtake()
    {
        _outtakeActive = true;
        _motorIntake.setPower(-OUTTAKE_SPEED);
    }

    //--- Run outtake (reverse) at custom speed
    public void outtake(double power)
    {
        _outtakeActive = true;
        _motorIntake.setPower(-Math.abs(power));
    }

    //--- Stop the intake motor
    public void stop()
    {
        _intakeOn = false;
        _outtakeActive = false;
        _motorIntake.setPower(0);
    }

    //--- Check if intake is currently running
    public boolean isIntakeOn()
    {
        return _intakeOn;
    }

    //--- Check if outtake is currently active
    public boolean isOuttakeActive()
    {
        return _outtakeActive;
    }

    //--- Get detected ball colors by sensor position
    public BallColor getLeftBallColor() { return _leftBallColor; }
    public BallColor getCenterBallColor() { return _centerBallColor; }
    public BallColor getRightBallColor() { return _rightBallColor; }

    //--- Get detected ball colors by shooter position (1, 2, 3)
    //--- Shooter 1 = Left sensor, Shooter 2 = Center sensor, Shooter 3 = Right sensor
    public BallColor getShooterBallColor(int shooterNumber)
    {
        switch (shooterNumber)
        {
            case 1:
                return _leftBallColor;
            case 2:
                return _centerBallColor;
            case 3:
                return _rightBallColor;
            default:
                return BallColor.NONE;
        }
    }

    //--- Get all shooter ball colors as an array [shooter1, shooter2, shooter3]
    public BallColor[] getAllShooterBallColors()
    {
        return new BallColor[] { _leftBallColor, _centerBallColor, _rightBallColor };
    }

    //--- Check if a specific shooter has a ball (any color)
    public boolean shooterHasBall(int shooterNumber)
    {
        BallColor color = getShooterBallColor(shooterNumber);
        return color != BallColor.NONE;
    }

    //--- Check if a specific shooter has a specific color ball
    public boolean shooterHasColor(int shooterNumber, BallColor targetColor)
    {
        return getShooterBallColor(shooterNumber) == targetColor;
    }

    //--- Count how many shooters have balls
    public int getBallCount()
    {
        int count = 0;
        if (_leftBallColor != BallColor.NONE) count++;
        if (_centerBallColor != BallColor.NONE) count++;
        if (_rightBallColor != BallColor.NONE) count++;
        return count;
    }

    //--- Count how many shooters have a specific color ball
    public int getColorCount(BallColor targetColor)
    {
        int count = 0;
        if (_leftBallColor == targetColor) count++;
        if (_centerBallColor == targetColor) count++;
        if (_rightBallColor == targetColor) count++;
        return count;
    }

    //endregion

    //region --- Testing ---

    //--- Test color sensors - shows telemetry for tuning
    //--- Always runs detection and updates lights so testing works
    //--- regardless of intake state
    public void testColorSensors()
    {
        //--- Always detect balls and update lights in test mode
        detectBalls();
        updateLights();

        //--- Show detailed telemetry
        _telemetry.addLine("=== COLOR SENSOR TEST (5-sample avg) ===");

        if (_colorSensorLeft != null && _distanceSensorLeft != null)
        {
            int r = _colorSensorLeft.red();
            int g = _colorSensorLeft.green();
            int b = _colorSensorLeft.blue();
            double dist = _distanceSensorLeft.getDistance(DistanceUnit.MM);
            
            //--- Calculate ratios from averaged values
            double gToR = (_leftAvgR > 0) ? _leftAvgG / _leftAvgR : 0;
            double bToG = (_leftAvgG > 0) ? _leftAvgB / _leftAvgG : 0;

            _telemetry.addLine(String.format("--- LEFT SENSOR (max %.0fmm) ---", LEFT_DISTANCE_THRESHOLD_MM));
            _telemetry.addData("  Raw RGB", "R:%d G:%d B:%d", r, g, b);
            _telemetry.addData("  Avg RGB", "R:%.0f G:%.0f B:%.0f", _leftAvgR, _leftAvgG, _leftAvgB);
            _telemetry.addData("  G/R ratio", "%.2f (GREEN if >%.1f)", gToR, GREEN_RATIO_THRESHOLD);
            _telemetry.addData("  B/G ratio", "%.2f (PURPLE if >%.2f & G/R<%.1f)", bToG, PURPLE_BG_THRESHOLD, PURPLE_GR_MAX);
            
            //--- Update rolling min/max buffer
            if (_leftAvgDist > 0 && _leftAvgDist < 200) { // Ignore invalid readings
                _leftMinMaxBuffer[_leftMinMaxIndex] = _leftAvgDist;
                _leftMinMaxIndex = (_leftMinMaxIndex + 1) % MINMAX_BUFFER_SIZE;
                if (_leftMinMaxCount < MINMAX_BUFFER_SIZE) _leftMinMaxCount++;
            }
            double[] leftMinMax = getMinMax(_leftMinMaxBuffer, _leftMinMaxCount);
            _telemetry.addData("  Distance", "%.1f mm [avg %.1f min:%.1f max:%.1f]", dist, _leftAvgDist, leftMinMax[0], leftMinMax[1]);
            _telemetry.addData("  Ball", "%s%s", _leftBallColor, _leftIsSticky ? " (STICKY)" : "");;
        }
        else
        {
            _telemetry.addData("Left Sensor", "NOT CONNECTED");
        }

        if (_colorSensorCenter != null && _distanceSensorCenter != null)
        {
            int r = _colorSensorCenter.red();
            int g = _colorSensorCenter.green();
            int b = _colorSensorCenter.blue();
            double dist = _distanceSensorCenter.getDistance(DistanceUnit.MM);
            
            //--- Calculate ratios from averaged values
            double gToR = (_centerAvgR > 0) ? _centerAvgG / _centerAvgR : 0;
            double bToG = (_centerAvgG > 0) ? _centerAvgB / _centerAvgG : 0;

            _telemetry.addLine(String.format("--- CENTER SENSOR (max %.0fmm) ---", CENTER_DISTANCE_THRESHOLD_MM));
            _telemetry.addData("  Raw RGB", "R:%d G:%d B:%d", r, g, b);
            _telemetry.addData("  Avg RGB", "R:%.0f G:%.0f B:%.0f", _centerAvgR, _centerAvgG, _centerAvgB);
            _telemetry.addData("  G/R ratio", "%.2f (GREEN if >%.1f)", gToR, GREEN_RATIO_THRESHOLD);
            _telemetry.addData("  B/G ratio", "%.2f (PURPLE if >%.2f & G/R<%.1f)", bToG, PURPLE_BG_THRESHOLD, PURPLE_GR_MAX);
            
            //--- Update rolling min/max buffer
            if (_centerAvgDist > 0 && _centerAvgDist < 200) { // Ignore invalid readings
                _centerMinMaxBuffer[_centerMinMaxIndex] = _centerAvgDist;
                _centerMinMaxIndex = (_centerMinMaxIndex + 1) % MINMAX_BUFFER_SIZE;
                if (_centerMinMaxCount < MINMAX_BUFFER_SIZE) _centerMinMaxCount++;
            }
            double[] centerMinMax = getMinMax(_centerMinMaxBuffer, _centerMinMaxCount);
            _telemetry.addData("  Distance", "%.1f mm [avg %.1f min:%.1f max:%.1f]", dist, _centerAvgDist, centerMinMax[0], centerMinMax[1]);
            _telemetry.addData("  Ball", "%s%s", _centerBallColor, _centerIsSticky ? " (STICKY)" : "");;
        }
        else
        {
            _telemetry.addData("Center Sensor", "NOT CONNECTED");
        }

        if (_colorSensorRight != null && _distanceSensorRight != null)
        {
            int r = _colorSensorRight.red();
            int g = _colorSensorRight.green();
            int b = _colorSensorRight.blue();
            double dist = _distanceSensorRight.getDistance(DistanceUnit.MM);
            
            //--- Calculate ratios from averaged values
            double gToR = (_rightAvgR > 0) ? _rightAvgG / _rightAvgR : 0;
            double bToG = (_rightAvgG > 0) ? _rightAvgB / _rightAvgG : 0;

            _telemetry.addLine(String.format("--- RIGHT SENSOR (max %.0fmm) ---", RIGHT_DISTANCE_THRESHOLD_MM));
            _telemetry.addData("  Raw RGB", "R:%d G:%d B:%d", r, g, b);
            _telemetry.addData("  Avg RGB", "R:%.0f G:%.0f B:%.0f", _rightAvgR, _rightAvgG, _rightAvgB);
            _telemetry.addData("  G/R ratio", "%.2f (GREEN if >%.1f)", gToR, GREEN_RATIO_THRESHOLD);
            _telemetry.addData("  B/G ratio", "%.2f (PURPLE if >%.2f & G/R<%.1f)", bToG, PURPLE_BG_THRESHOLD, PURPLE_GR_MAX);
            
            //--- Update rolling min/max buffer
            if (_rightAvgDist > 0 && _rightAvgDist < 200) { // Ignore invalid readings
                _rightMinMaxBuffer[_rightMinMaxIndex] = _rightAvgDist;
                _rightMinMaxIndex = (_rightMinMaxIndex + 1) % MINMAX_BUFFER_SIZE;
                if (_rightMinMaxCount < MINMAX_BUFFER_SIZE) _rightMinMaxCount++;
            }
            double[] rightMinMax = getMinMax(_rightMinMaxBuffer, _rightMinMaxCount);
            _telemetry.addData("  Distance", "%.1f mm [avg %.1f min:%.1f max:%.1f]", dist, _rightAvgDist, rightMinMax[0], rightMinMax[1]);
            _telemetry.addData("  Ball", "%s%s", _rightBallColor, _rightIsSticky ? " (STICKY)" : "");
        }
        else
        {
            _telemetry.addData("Right Sensor", "NOT INSTALLED");
        }

        _telemetry.addLine("");
        _telemetry.addData("Detection", "GREEN: G/R>%.1f | PURPLE: B/G>%.2f & G/R<%.1f", 
            GREEN_RATIO_THRESHOLD, PURPLE_BG_THRESHOLD, PURPLE_GR_MAX);
    }

    //endregion

    //region --- Telemetry ---

    public void getTelemetry()
    {
        if (_showInfo)
        {
            _telemetry.addData("Intake Power", "%4.2f", _motorIntake.getPower());
            _telemetry.addData("Intake On", _intakeOn);
            _telemetry.addData("Outtake Active", _outtakeActive);

            //--- Ball detection telemetry
            _telemetry.addData("Ball Left", _leftBallColor);
            _telemetry.addData("Ball Center", _centerBallColor);
            _telemetry.addData("Ball Right", _rightBallColor);

            //--- Raw sensor values for debugging
            if (_colorSensorLeft != null && _distanceSensorLeft != null)
            {
                _telemetry.addData("Left RGB", "R:%d G:%d B:%d", 
                    _colorSensorLeft.red(), _colorSensorLeft.green(), _colorSensorLeft.blue());
                _telemetry.addData("Left Dist", "%.1f mm", 
                    _distanceSensorLeft.getDistance(DistanceUnit.MM));
            }
            if (_colorSensorCenter != null && _distanceSensorCenter != null)
            {
                _telemetry.addData("Center RGB", "R:%d G:%d B:%d", 
                    _colorSensorCenter.red(), _colorSensorCenter.green(), _colorSensorCenter.blue());
                _telemetry.addData("Center Dist", "%.1f mm", 
                    _distanceSensorCenter.getDistance(DistanceUnit.MM));
            }
        }
    }

    //endregion
}
