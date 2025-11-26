package org.firstinspires.ftc.teamcode.hardware;

//region --- Imports ---
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
//endregion

public class Lights
{
    //region --- Constants ---
    //--- goBILDA Indicator Light Position Map
    public static final double COLOR_OFF = 0.0;
    public static final double COLOR_RED = 0.29;
    public static final double COLOR_YELLOW = 0.38;
    public static final double COLOR_GREEN = 0.51;
    public static final double COLOR_BLUE = 0.61;
    public static final double COLOR_PURPLE = 0.69;
    public static final double COLOR_WHITE = 1.0;
    //endregion

    //region --- Enums ---
    public enum Color
    {
        OFF(COLOR_OFF),
        RED(COLOR_RED),
        YELLOW(COLOR_YELLOW),
        GREEN(COLOR_GREEN),
        BLUE(COLOR_BLUE),
        PURPLE(COLOR_PURPLE),
        WHITE(COLOR_WHITE);

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

    //endregion

    //region --- Hardware ---
    private final Servo _servoLightLeft;
    private final Servo _servoLightMiddle;
    private final Servo _servoLightRight;
    private final Gamepad _gamepad1;
    private final Gamepad _gamepad2;
    private final Telemetry _telemetry;
    private final boolean _showInfo;

    private int _robotVersion;
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
        //--- Set default light colors on initialization
        setAll(Color.OFF);
    }
    //endregion

    //region --- Public Methods - Set All Lights ---

    //--- Set all lights to the same color enum
    public void setAll(Color color)
    {
        setAll(color.getPosition());
    }

    //--- Set all lights to the same raw position value
    public void setAll(double positionValue)
    {
        setLeft(positionValue);
        setMiddle(positionValue);
        setRight(positionValue);
    }

    //endregion

    //region --- Public Methods - Set Individual Lights by Color Enum ---

    public void setLeft(Color color)
    {
        setLeft(color.getPosition());
    }

    public void setMiddle(Color color)
    {
        setMiddle(color.getPosition());
    }

    public void setRight(Color color)
    {
        setRight(color.getPosition());
    }

    //endregion

    //region --- Public Methods - Set Individual Lights by Position Value ---

    public void setLeft(double positionValue)
    {
        _servoLightLeft.setPosition(positionValue);
    }

    public void setMiddle(double positionValue)
    {
        _servoLightMiddle.setPosition(positionValue);
    }

    public void setRight(double positionValue)
    {
        _servoLightRight.setPosition(positionValue);
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

    //region --- Telemetry ---

    public void getTelemetry()
    {
        if (_showInfo)
        {
            _telemetry.addData("Light Left Position", "%4.2f", _servoLightLeft.getPosition());
            _telemetry.addData("Light Middle Position", "%4.2f", _servoLightMiddle.getPosition());
            _telemetry.addData("Light Right Position", "%4.2f", _servoLightRight.getPosition());
        }
    }

    //endregion
}
