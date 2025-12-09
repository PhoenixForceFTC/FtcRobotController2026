package org.firstinspires.ftc.teamcode;

//region --- Imports ---
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
//endregion

//region --- Controls ---
//======================================================================
// RPM TUNING TELEOP - Build RPM lookup tables for 1/2/3 ball shots
//======================================================================
//
// GAMEPAD 1 CONTROLS:
//
//  DISTANCE CONTROL:
//  - Dpad Up           - Move CLOSER to target (decrease distance by 1")
//                        • If camera visible: drives forward until camera reads target
//                        • If no camera: manually increment target distance
//  - Dpad Down         - Move FARTHER from target (increase distance by 1")
//                        • If camera visible: drives backward until camera reads target
//                        • If no camera: manually decrement target distance
//
//  RPM CONTROL:
//  - Y (▲)             - Increase RPM by 50
//  - A (✕)             - Decrease RPM by 50
//
//  INTAKE:
//  - Left Trigger      - Toggle intake on/off
//                        • Auto-stops after 3 balls detected
//                        • Outtakes briefly then stops
//  - Left Bumper       - Outtake for 2 seconds
//
//  FIRING:
//  - Right Trigger     - Fire balls (uses intake ball count: 1, 2, or 3)
//                        • Shows exact RPM used when firing
//
//  RECORDING:
//  - B (○)             - Record SUCCESS - saves current RPM to table
//                        • Records for current distance and ball count
//
//  DISPLAY:
//  - X (■)             - Toggle between table pages (1-40, 41-80, 81-120)
//
//======================================================================
// TELEMETRY DISPLAY:
//======================================================================
//
//        [1b]    [2b]    [3b]
//   1" [----] [----] [----]
//   2" [2200] [2300] [2400]
//   3" [2200] [2300] [----]
//   ...
//
// Where [----] means no value recorded yet
//
//======================================================================
//endregion

@TeleOp(name="RPM Tuning", group="2")
public class TeleOp_RPMTuning extends LinearOpMode
{
    //region --- Constants ---
    private static final double RPM_INCREMENT = 50.0;
    private static final double RPM_MIN = 2000.0;
    private static final double RPM_MAX = 4500.0;
    private static final double DEFAULT_RPM = 2500.0;
    
    private static final double DISTANCE_DEADBAND = 0.5;  // Inches - stop when within this range
    private static final double DRIVE_SPEED = 0.15;       // Slow speed for precise positioning
    
    private static final double DPAD_DEBOUNCE_TIME = 0.3; // Seconds between dpad presses
    private static final double BUTTON_DEBOUNCE_TIME = 0.2; // Seconds between button presses
    
    private static final int TABLE_SIZE = 120;            // 1" to 120"
    private static final int ROWS_PER_PAGE = 20;          // Show 20 rows at a time
    //endregion

    //region --- Variables ---
    RobotHardware _robot = new RobotHardware(this);
    private ElapsedTime _runtime = new ElapsedTime();
    
    //--- RPM Tables (modifiable during runtime)
    //--- -1 means not yet recorded
    private double[] _rpmTable1Ball = new double[TABLE_SIZE];
    private double[] _rpmTable2Balls = new double[TABLE_SIZE];
    private double[] _rpmTable3Balls = new double[TABLE_SIZE];
    
    //--- Current state
    private int _targetDistanceInches = 1;    // 1-120
    private double _currentRPM = DEFAULT_RPM;
    private int _lastFiredBallCount = 0;
    private double _lastFiredRPM = 0;
    private boolean _lastShotSuccess = false;
    
    //--- Display state
    private int _displayPage = 0;             // 0=1-40, 1=41-80, 2=81-120
    
    //--- Movement state
    private boolean _isMoving = false;
    private boolean _movingCloser = false;    // true = forward, false = backward
    
    //--- Debounce timers
    private ElapsedTime _dpadTimer = new ElapsedTime();
    private ElapsedTime _yButtonTimer = new ElapsedTime();
    private ElapsedTime _aButtonTimer = new ElapsedTime();
    private ElapsedTime _bButtonTimer = new ElapsedTime();
    private ElapsedTime _xButtonTimer = new ElapsedTime();
    private ElapsedTime _triggerTimer = new ElapsedTime();
    
    //--- Debounce flags
    private boolean _dpadUpPressed = false;
    private boolean _dpadDownPressed = false;
    private boolean _yPressed = false;
    private boolean _aPressed = false;
    private boolean _bPressed = false;
    private boolean _xPressed = false;
    private boolean _triggerPressed = false;
    //endregion

    //region --- OpMode ---
    @Override
    public void runOpMode()
    {
        //--- Initialize tables to -1 (not recorded)
        for (int i = 0; i < TABLE_SIZE; i++)
        {
            _rpmTable1Ball[i] = -1;
            _rpmTable2Balls[i] = -1;
            _rpmTable3Balls[i] = -1;
        }
        
        //--- Robot Initialize
        int robotVersion = 2;
        _robot.init(robotVersion);
        
        //--- Display and wait for start
        telemetry.addData("STATUS", "RPM Tuning Ready");
        telemetry.addData("", "Position robot touching target (0\")");
        telemetry.addData("", "Press START to begin");
        telemetry.update();
        waitForStart();
        _runtime.reset();
        
        //--- Hardware Initialize
        _robot.intake.initialize();
        _robot.lights.initialize();
        _robot.camera.setModeTeleOp();
        
        //--- Reset timers
        _dpadTimer.reset();
        _yButtonTimer.reset();
        _aButtonTimer.reset();
        _bButtonTimer.reset();
        _xButtonTimer.reset();
        _triggerTimer.reset();
        
        //--- Main loop
        while (opModeIsActive())
        {
            //--- Run hardware updates
            _robot.run();
            
            //--- Handle controls
            handleDistanceControl();
            handleRPMControl();
            handleFiring();
            handleRecording();
            handleDisplayToggle();
            
            //--- Handle intake (left trigger toggle, auto-stop on 3 balls)
            _robot.intake.controlIntake();
            
            //--- Update movement if active
            updateMovement();
            
            //--- Display telemetry
            displayTelemetry();
            
            telemetry.update();
        }
        
        //--- Stop motors on exit
        _robot.drive.stopMotors();
        _robot.flywheel.stop();
    }
    //endregion

    //region --- Distance Control ---
    private void handleDistanceControl()
    {
        //--- Dpad Up - Move CLOSER (decrease distance)
        if (gamepad1.dpad_up)
        {
            if (!_dpadUpPressed && _dpadTimer.seconds() >= DPAD_DEBOUNCE_TIME)
            {
                _dpadUpPressed = true;
                _dpadTimer.reset();
                
                if (_targetDistanceInches > 1)
                {
                    _targetDistanceInches--;
                    
                    //--- Check if camera has a reading
                    double cameraDistance = _robot.camera.getDistanceInches();
                    if (cameraDistance > 0)
                    {
                        //--- Start moving toward target distance
                        _isMoving = true;
                        _movingCloser = true;
                    }
                    //--- If no camera, just update target (use tape measure)
                }
            }
        }
        else
        {
            _dpadUpPressed = false;
        }
        
        //--- Dpad Down - Move FARTHER (increase distance)
        if (gamepad1.dpad_down)
        {
            if (!_dpadDownPressed && _dpadTimer.seconds() >= DPAD_DEBOUNCE_TIME)
            {
                _dpadDownPressed = true;
                _dpadTimer.reset();
                
                if (_targetDistanceInches < TABLE_SIZE)
                {
                    _targetDistanceInches++;
                    
                    //--- Check if camera has a reading
                    double cameraDistance = _robot.camera.getDistanceInches();
                    if (cameraDistance > 0)
                    {
                        //--- Start moving toward target distance
                        _isMoving = true;
                        _movingCloser = false;
                    }
                    //--- If no camera, just update target (use tape measure)
                }
            }
        }
        else
        {
            _dpadDownPressed = false;
        }
    }
    
    private void updateMovement()
    {
        if (!_isMoving) return;
        
        //--- Get current camera reading
        double cameraDistance = _robot.camera.getDistanceInches();
        
        //--- If camera lost tracking, stop immediately
        if (cameraDistance < 0)
        {
            _isMoving = false;
            _robot.drive.stopMotors();
            return;
        }
        
        //--- Check if we've reached target distance (within deadband)
        double error = cameraDistance - _targetDistanceInches;
        
        if (Math.abs(error) <= DISTANCE_DEADBAND)
        {
            //--- Reached target, stop
            _isMoving = false;
            _robot.drive.stopMotors();
            return;
        }
        
        //--- Move toward target
        if (_movingCloser)
        {
            //--- Moving closer (forward) - camera distance should decrease
            if (cameraDistance > _targetDistanceInches)
            {
                _robot.drive.driveForward(DRIVE_SPEED);
            }
            else
            {
                //--- Overshot, stop
                _isMoving = false;
                _robot.drive.stopMotors();
            }
        }
        else
        {
            //--- Moving farther (backward) - camera distance should increase
            if (cameraDistance < _targetDistanceInches)
            {
                _robot.drive.driveBackward(DRIVE_SPEED);
            }
            else
            {
                //--- Overshot, stop
                _isMoving = false;
                _robot.drive.stopMotors();
            }
        }
    }
    //endregion

    //region --- RPM Control ---
    private void handleRPMControl()
    {
        //--- Y button - Increase RPM
        if (gamepad1.y)
        {
            if (!_yPressed && _yButtonTimer.seconds() >= BUTTON_DEBOUNCE_TIME)
            {
                _yPressed = true;
                _yButtonTimer.reset();
                
                _currentRPM = Math.min(RPM_MAX, _currentRPM + RPM_INCREMENT);
                _robot.flywheel.setVelocity(_currentRPM);
            }
        }
        else
        {
            _yPressed = false;
        }
        
        //--- A button - Decrease RPM
        if (gamepad1.a)
        {
            if (!_aPressed && _aButtonTimer.seconds() >= BUTTON_DEBOUNCE_TIME)
            {
                _aPressed = true;
                _aButtonTimer.reset();
                
                _currentRPM = Math.max(RPM_MIN, _currentRPM - RPM_INCREMENT);
                _robot.flywheel.setVelocity(_currentRPM);
            }
        }
        else
        {
            _aPressed = false;
        }
    }
    //endregion

    //region --- Firing ---
    private void handleFiring()
    {
        //--- Right Trigger - Fire
        if (gamepad1.right_trigger > 0.5)
        {
            if (!_triggerPressed && _triggerTimer.seconds() >= BUTTON_DEBOUNCE_TIME)
            {
                _triggerPressed = true;
                _triggerTimer.reset();
                
                //--- Get ball count from intake
                int ballCount = _robot.intake.getBallCount();
                if (ballCount <= 0) ballCount = 1;  // Default to 1 if unknown
                if (ballCount > 3) ballCount = 3;   // Max 3
                
                //--- Store what we're firing for display
                _lastFiredBallCount = ballCount;
                _lastFiredRPM = _currentRPM;
                _lastShotSuccess = false;  // Will be set by B button
                
                //--- Make sure flywheel is at speed
                _robot.flywheel.setVelocity(_currentRPM);
                
                //--- Fire all kickers
                _robot.kickers.fireAll();
            }
        }
        else
        {
            _triggerPressed = false;
        }
    }
    //endregion

    //region --- Recording ---
    private void handleRecording()
    {
        //--- B button - Record SUCCESS
        if (gamepad1.b)
        {
            if (!_bPressed && _bButtonTimer.seconds() >= BUTTON_DEBOUNCE_TIME)
            {
                _bPressed = true;
                _bButtonTimer.reset();
                
                //--- Record current RPM for current distance and last fired ball count
                if (_lastFiredBallCount > 0 && _targetDistanceInches >= 1 && _targetDistanceInches <= TABLE_SIZE)
                {
                    int index = _targetDistanceInches - 1;  // 0-indexed
                    
                    switch (_lastFiredBallCount)
                    {
                        case 1:
                            _rpmTable1Ball[index] = _lastFiredRPM;
                            break;
                        case 2:
                            _rpmTable2Balls[index] = _lastFiredRPM;
                            break;
                        case 3:
                            _rpmTable3Balls[index] = _lastFiredRPM;
                            break;
                    }
                    
                    _lastShotSuccess = true;
                }
            }
        }
        else
        {
            _bPressed = false;
        }
    }
    //endregion

    //region --- Display Toggle ---
    private void handleDisplayToggle()
    {
        //--- X button - Toggle display page
        if (gamepad1.x)
        {
            if (!_xPressed && _xButtonTimer.seconds() >= BUTTON_DEBOUNCE_TIME)
            {
                _xPressed = true;
                _xButtonTimer.reset();
                
                _displayPage = (_displayPage + 1) % 6;  // 0-5 for 20 rows each
            }
        }
        else
        {
            _xPressed = false;
        }
    }
    //endregion

    //region --- Telemetry ---
    private void displayTelemetry()
    {
        //--- Header
        telemetry.addData("=== RPM TUNING ===", "");
        
        //--- Current state
        double cameraDistance = _robot.camera.getDistanceInches();
        String cameraStatus = (cameraDistance > 0) ? String.format("%.1f\"", cameraDistance) : "NO TAG";
        String storedDistance = _robot.camera.getStoredDistanceFormatted();
        
        telemetry.addData("Target Distance", "%d\"", _targetDistanceInches);
        telemetry.addData("Camera Reading", cameraStatus);
        telemetry.addData("Last Distance", storedDistance);
        telemetry.addData("Current RPM", "%.0f", _currentRPM);
        telemetry.addData("Flywheel Actual", "%.0f RPM", _robot.flywheel.getCurrentRPM());
        telemetry.addData("Ball Count", _robot.intake.getBallCount());
        
        if (_isMoving)
        {
            telemetry.addData("STATUS", "MOVING %s", _movingCloser ? "CLOSER" : "FARTHER");
        }
        
        //--- Last shot info
        if (_lastFiredBallCount > 0)
        {
            telemetry.addData("Last Shot", "%d balls @ %.0f RPM %s", 
                    _lastFiredBallCount, _lastFiredRPM, 
                    _lastShotSuccess ? "✓ RECORDED" : "(press B to record)");
        }
        
        //--- Controls reminder
        telemetry.addData("---", "---");
        telemetry.addData("Dpad", "Up=Closer, Down=Farther");
        telemetry.addData("Y/A", "RPM +/- 50");
        telemetry.addData("RT", "Fire | B=Record | X=Page");
        
        //--- Table display
        telemetry.addData("---", "---");
        int startRow = _displayPage * ROWS_PER_PAGE + 1;
        int endRow = Math.min(startRow + ROWS_PER_PAGE - 1, TABLE_SIZE);
        telemetry.addData("TABLE", "Page %d (%d\"-%d\")", _displayPage + 1, startRow, endRow);
        telemetry.addData("    ", "  [1b]   [2b]   [3b]");
        
        for (int inch = startRow; inch <= endRow; inch++)
        {
            int index = inch - 1;
            String v1 = formatRPM(_rpmTable1Ball[index]);
            String v2 = formatRPM(_rpmTable2Balls[index]);
            String v3 = formatRPM(_rpmTable3Balls[index]);
            
            //--- Highlight current target distance
            String prefix = (inch == _targetDistanceInches) ? ">>>" : "   ";
            telemetry.addData(prefix + String.format("%3d\"", inch), "%s %s %s", v1, v2, v3);
        }
    }
    
    private String formatRPM(double rpm)
    {
        if (rpm < 0)
        {
            return "[----]";
        }
        else
        {
            return String.format("[%4.0f]", rpm);
        }
    }
    //endregion
}
