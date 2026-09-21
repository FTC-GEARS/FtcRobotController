package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.function.Supplier;

public class intakeV3 {

    private static DcMotorEx intake;
    Supplier button = null;




    public void initIntake(HardwareMap hardwareMap){

        intake = hardwareMap.get(DcMotorEx.class, "intake");

    }
    private enum MotorState{
        MOTOR_ON(1.0),
        MOTOR_OFF(0.0);

        public final double power;

        MotorState(double power) {
            this.power = power;
        }
    }

    private MotorState currentState = MotorState.MOTOR_OFF;

    public void toggleState(){

        if (currentState == MotorState.MOTOR_OFF) {
            currentState = MotorState.MOTOR_ON;
        } else {
            currentState = MotorState.MOTOR_OFF;
        }
    }


}