package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeV3 {

    private DcMotorEx intake;
    private double maxIntake = 1.0;

    public enum MotorState {
        MOTOR_ON,
        MOTOR_OFF
    }

    private MotorState currentState = MotorState.MOTOR_OFF;

    public void initIntake(HardwareMap hardwareMap, double maxIntake) {
        this.maxIntake = maxIntake;
        intake = hardwareMap.get(DcMotorEx.class, "intake");
    }

    public void toggleState() {
        if (currentState == MotorState.MOTOR_OFF) {
            currentState = MotorState.MOTOR_ON;
            intake.setPower(maxIntake);
        } else {
            currentState = MotorState.MOTOR_OFF;
            intake.setPower(0.0);
        }
    }
}
