package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MainGuts {

    private DcMotorEx shooter, passer;

    private double shooterRPM = 3000;
    private double maxPasser = 1.0;

    public enum PasserState {
        MOTOR_ON,
        MOTOR_OFF
    }

    public enum ShooterState {
        MOTOR_ON,
        MOTOR_OFF
    }

    private PasserState passerState = PasserState.MOTOR_OFF;
    private ShooterState shooterState = ShooterState.MOTOR_OFF;

    public void initGuts(HardwareMap hardwareMap, double maxPasser, double shooterRPM) {
        this.maxPasser = maxPasser;
        this.shooterRPM = shooterRPM;

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        passer = hardwareMap.get(DcMotorEx.class, "transfer");

        shooter.setDirection(DcMotor.Direction.REVERSE);
        passer.setDirection(DcMotor.Direction.REVERSE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        passer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void updateShooter(double shooterRPM) {
        this.shooterRPM = shooterRPM;
        if (shooterState == ShooterState.MOTOR_ON) {
            setShooterOn();
        }
    }

    private void setShooterOn() {
        shooter.setPower(1.0);
    }

    public void togglePasser() {
        if (passerState == PasserState.MOTOR_OFF) {
            passerState = PasserState.MOTOR_ON;
            passer.setPower(maxPasser);
        } else {
            passerState = PasserState.MOTOR_OFF;
            passer.setPower(0.0);
        }
    }

    public void toggleShooter() {
        if (shooterState == ShooterState.MOTOR_OFF) {
            shooterState = ShooterState.MOTOR_ON;
            setShooterOn();
        } else {
            shooterState = ShooterState.MOTOR_OFF;
            shooter.setPower(0.0);
        }
    }

    public PasserState getPasserState() {
        return passerState;
    }

    public ShooterState getShooterState() {
        return shooterState;
    }

    public double getPasserPower() {
        return passer != null ? passer.getPower() : 0.0;
    }

    public double getShooterPower() {
        return shooter != null ? shooter.getPower() : 0.0;
    }
}
