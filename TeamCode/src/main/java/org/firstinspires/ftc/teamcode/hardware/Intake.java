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
    private static final double OUTTAKE_DURATION = 1.0;

    //--- Ball Detection Settings (per-sensor distance thresholds)
    //--- Each sensor may have different mounting distances
    private static final double LEFT_DISTANCE_THRESHOLD_MM = 80.0;    // Left sensor: ball if < 80mm
    private static final double CENTER_DISTANCE_THRESHOLD_MM = 95.0;  // Center sensor: ball if < 95mm
    private static final double RIGHT_DISTANCE_THRESHOLD_MM = 80.0;   // Right sensor: TBD

    //--- Averaging settings
    private static final int AVERAGING_SAMPLES = 5;  // Number of samples to average

    //--- Color Detection Thresholds (REV Color Sensor V3)
    //--- Using RGB ratios for reliable detection
    //--- Background (no ball) has G/R ~1.4, so we need higher thresholds
    //--- Green balls: G/R > 2.5 (your readings show 3.0+)
    //--- Purple balls: B/G > 0.85 AND G/R < 1.8 (distinguishes from green and background)
    private static final double GREEN_RATIO_THRESHOLD = 2.5;   // G/R must exceed this for green
    private static final double PURPLE_BG_THRESHOLD = 0.85;    // B/G must exceed this for purple
    private static final double PURPLE_GR_MAX = 1.8;           // G/R must be below this for purple
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
    //endregion

    //region --- State ---
    private boolean _intakeOn = false;
    private boolean _triggerWasPressed = false;
    private boolean _outtakeActive = false;
    private ElapsedTime _outtakeTimer = new ElapsedTime();

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
        stop();
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
    //endregion

    //region --- Run (call this in your main loop) ---
    public void run()
    {
        //--- Only detect balls and update lights while intaking or outtaking
        if (isIntakeOn() || isOuttakeActive())
        {
            detectBalls();
            updateLights();
        }
    }
    //endregion

    //region --- Ball Detection ---

    private void detectBalls()
    {
        //--- Update sensor buffers and detect balls using averaged values
        _leftBallColor = updateAndDetect(
            _colorSensorLeft, _distanceSensorLeft,
            _leftDistBuffer, _leftRedBuffer, _leftGreenBuffer, _leftBlueBuffer,
            _leftBufferIndex, LEFT_DISTANCE_THRESHOLD_MM, "left");
        _leftBufferIndex = (_leftBufferIndex + 1) % AVERAGING_SAMPLES;

        _centerBallColor = updateAndDetect(
            _colorSensorCenter, _distanceSensorCenter,
            _centerDistBuffer, _centerRedBuffer, _centerGreenBuffer, _centerBlueBuffer,
            _centerBufferIndex, CENTER_DISTANCE_THRESHOLD_MM, "center");
        _centerBufferIndex = (_centerBufferIndex + 1) % AVERAGING_SAMPLES;

        _rightBallColor = updateAndDetect(
            _colorSensorRight, _distanceSensorRight,
            _rightDistBuffer, _rightRedBuffer, _rightGreenBuffer, _rightBlueBuffer,
            _rightBufferIndex, RIGHT_DISTANCE_THRESHOLD_MM, "right");
        _rightBufferIndex = (_rightBufferIndex + 1) % AVERAGING_SAMPLES;
    }

    private BallColor updateAndDetect(
        ColorSensor colorSensor, DistanceSensor distanceSensor,
        double[] distBuffer, int[] redBuffer, int[] greenBuffer, int[] blueBuffer,
        int bufferIndex, double distanceThreshold, String sensorName)
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
        if (avgDist > distanceThreshold)
        {
            return BallColor.NONE;
        }

        //--- Ball is present (under distance threshold), determine color
        //--- Calculate ratios from averaged values (with safety for divide by zero)
        double gToR = (avgR > 0) ? avgG / avgR : 0;
        double bToG = (avgG > 0) ? avgB / avgG : 0;

        //--- Green ball: G/R > 2.5 (background is ~1.4, green balls are 3.0+)
        if (gToR > GREEN_RATIO_THRESHOLD)
        {
            return BallColor.GREEN;
        }

        //--- Purple ball: B/G > 0.85 AND G/R < 1.8
        if (bToG > PURPLE_BG_THRESHOLD && gToR < PURPLE_GR_MAX)
        {
            return BallColor.PURPLE;
        }

        //--- Ball detected (under distance threshold) but color unclear → show white light
        return BallColor.UNKNOWN;
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
    public void restoreLights()
    {
        if (_lights != null)
        {
            _lights.initialize();  // Reset to default colors
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

    //--- Run intake at configured speed
    public void intake()
    {
        _intakeOn = true;
        _motorIntake.setPower(INTAKE_SPEED);
    }

    //--- Run intake at custom speed
    public void intake(double power)
    {
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

    //--- Get detected ball colors
    public BallColor getLeftBallColor() { return _leftBallColor; }
    public BallColor getCenterBallColor() { return _centerBallColor; }
    public BallColor getRightBallColor() { return _rightBallColor; }

    //endregion

    //region --- Testing ---

    //--- Test color sensors without intake running
    //--- Call this in your main loop to see raw sensor values
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
            _telemetry.addData("  Distance", "%.1f mm (avg %.1f)", dist, _leftAvgDist);
            _telemetry.addData("  Ball", _leftBallColor);
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
            _telemetry.addData("  Distance", "%.1f mm (avg %.1f)", dist, _centerAvgDist);
            _telemetry.addData("  Ball", _centerBallColor);
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
            _telemetry.addData("  Distance", "%.1f mm (avg %.1f)", dist, _rightAvgDist);
            _telemetry.addData("  Ball", _rightBallColor);
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
