package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class PIDF {
    private double kP, kI, kD, kF;
    private double targetPosition = 0;
    private double integralSum = 0;
    private double lastError = 0;

    private ElapsedTime timer = new ElapsedTime();

    // Constructor to initialize coefficients
    public PIDF(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        timer.reset();
    }

    public void setTargetPosition(double target) {
        this.targetPosition = target;
    }

    // Call this inside your main OpMode loop
    public double calculate(double currentPosition) {
        double error = targetPosition - currentPosition;
        double deltaTime = timer.seconds();

        // Reset timer for next loop iteration
        timer.reset();

        // Proportional Term
        double pTerm = kP * error;

        // Integral Term (accumulates error over time)
        integralSum += error * deltaTime;
        // Anti-windup: cap the integral sum to avoid massive overshoots
        if (Math.abs(error) < 1) { // Stop integrating if error is tiny
            integralSum = 0;
        }
        double iTerm = kI * integralSum;

        // Derivative Term (predicts future error based on rate of change)
        double dTerm = 0;
        if (deltaTime > 0) {
            dTerm = kD * ((error - lastError) / deltaTime);
        }
        lastError = error;

        // Feedforward Term (provides a constant baseline power to counteract gravity/friction)
        double fTerm = kF;
        // Note: For lifts/arms, you may want to multiply kF by Math.cos(angle) or use a static value.

        // Combined output power
        return pTerm + iTerm + dTerm + fTerm;
    }
}

