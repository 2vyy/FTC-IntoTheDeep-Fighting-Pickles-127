package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

// implementation of PIDF Control for a motor.
// Refer to https://youtu.be/4Y7zG48uHRo?si=2XhRVeDYf7zzIsRo for a conceptual introduction
// Refer to https://www.ctrlaltftc.com/ for more in-depth concepts and code explanations
public class PIDFController {
    private double Kp = 0;
    private double Ki = 0;
    private double Kd = 0;
    private double Kf = 0; //Kf basically counteracts the force of gravity so the position stays even when power = 0

    //Remember: power is between -1 and 1 for a motor. Everything should be a double to allow for decimals.

    private double lastError = 0;
    private double integralSum = 0;
    private ElapsedTime timer = new ElapsedTime();

    //object constructor. Takes in P, I, and D constants tuned for that specific motor. refer to https://youtu.be/E6H6Nqe6qJo?si=nHZ2z9U3J81rZ-2E&t=438 for tuning
    public PIDFController(double _Kp, double _Ki, double _Kd, double _Kf) {
        Kp = _Kp;
        Ki = _Ki;
        Kd = _Kd;
        Kf = _Kf;
    }

    // params: target is goal position, state is current position (ticks)
    // returns a double to be assigned to power
    public double calculate(double target, double state) {
        // calculate the statistical error
        double error = target - state;

        // rate of change of the error, simple displacement over time
        double derivative = (error - lastError) / timer.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative) + Kf;

        lastError = error;

        // reset the timer for next time
        timer.reset();
        return out;
    }

    public double getLastError() {
        return lastError;
    }
}