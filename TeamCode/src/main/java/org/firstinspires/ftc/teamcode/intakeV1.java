package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "Robot:intakeV1", group = "Robot")
public class intakeV1 extends OpMode {
    private DcMotor intake = null;
    private MotorState currentState;

    private enum MotorState {
        MOTOR_ON,
        MOTOR_OFF
    }

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        //MotorState currentState = MotorState.MOTOR_OFF;
        currentState = MotorState.MOTOR_OFF;
    }
    @Override
    public void loop() {

        if (gamepad1.a){
            switch (currentState) {
                case MOTOR_ON:
                    currentState = MotorState.MOTOR_OFF;
                    intake.setPower(0);
                    break;
                case MOTOR_OFF:
                    currentState = MotorState.MOTOR_ON;
                    intake.setPower(1);
                    break;
            }
        }

    }
}
