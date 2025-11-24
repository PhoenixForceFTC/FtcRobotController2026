package org.firstinspires.ftc.teamcode.utils;

//region --- Imports ---
import com.qualcomm.robotcore.hardware.Servo;
//endregion

public class ServoUtils
{
    //--- Moves the servo to the specified position and holds it
    public static double moveToPosition(Servo servo, double position) {
        position = clampPosition(position);
        servo.setPosition(position);
        //--- The servo will maintain this position as PWM remains enabled
        return servo.getPosition();
    }

    //--- Moves the servo to the specified position and disables PWM after a delay
    public static double moveToPositionAndDisable(Servo servo, double position, int delayMs)
    {
        //--- Create a new thread to handle the servo movement and PWM disable
        new Thread(() ->
        {
            try
            {
                servo.setPosition(clampPosition(position));
                //--- Wait for the servo to reach the desired position
                sleep(delayMs); //--- Adjust based on servo speed
            }
            finally
            {
                //--- Disable PWM to stop holding the position
                servo.getController().pwmDisable();
            }
        }).start();

        return servo.getPosition();
    }

    //--- Incrementally adjust the servo position
    public static double incrementPosition(Servo servo, double increment) {
        double newPosition = clampPosition(servo.getPosition() + increment);
        servo.setPosition(newPosition);

        return servo.getPosition();
    }

    public static void disable(Servo servo)
    {
        servo.getController().pwmDisable();
    }

    private static double clampPosition(double position) {
        return Math.max(0.0, Math.min(1.0, position));
    }


    //--- TODO: Move to util class
    private static void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
