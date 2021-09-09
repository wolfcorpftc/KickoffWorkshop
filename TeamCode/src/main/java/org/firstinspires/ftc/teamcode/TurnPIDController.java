package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

// Single use per object
public class TurnPIDController {
    private double kP, kI, kD;
    private ElapsedTime timer = new ElapsedTime();
    private double targetAngle;
    private double lastError = 0;
    private double accumulatedError = 0;
    private double lastTime = -1;
    private double lastSlope = 0;

    public TurnPIDController(double target, double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
        targetAngle = target;
    }

    public double update(double currentAngle) {
        // TODO: make sure angles are within bounds and are in same format (e.g., 0 <= | angle | <= 180)
        //   and ensure direction is correct

        // P
        double error = targetAngle - currentAngle;

        // I
        accumulatedError += error;
        if (Math.abs(error) < 0.1) {
            accumulatedError = 0;
        }

        // D
        double slope = 0;
        if (lastTime > 0) {
            slope = (error - lastError) / (timer.milliseconds() - lastTime);
        }
        lastSlope = slope;
        lastTime = timer.milliseconds();

        double motorPower = (error < 0 ? -0.1 : 0.1) +
                0.9 * Math.tanh(kP * error + kI * accumulatedError - kD * slope);
        return Math.abs(motorPower) < 0.104 ? 0 : motorPower;
    }

    double getLastSlope() {
        return lastSlope;
    }
}
