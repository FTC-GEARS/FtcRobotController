package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "Robot:intakeV1", group = "Robot")
public class intakeV1 extends OpMode {
    private DcMotor intake = null;
    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
    }
    @Override
    public void loop() {
        if (gamepad1.a) {
            intake.setPower(1);
        }
    }
}
