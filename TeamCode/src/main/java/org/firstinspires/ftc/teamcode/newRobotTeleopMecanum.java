package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "Robot: USE THIS MECANUM", group = "Robot")
public class newRobotTeleopMecanum extends OpMode {

    mecanumDriveFunctions MDF = new mecanumDriveFunctions();
    intakeV3 intake = new intakeV3();
    targetLock TL = new targetLock();
    @Override
    public void init() {
    MDF.initMecanum(hardwareMap);
    intake.initIntake(hardwareMap, gamepad1.a);
    }

    IMU imu;
    @Override
    public void loop() {
        telemetry.addLine("Press x to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");
        telemetry.addLine("Aim robot the robot direction as you and press x to reset yaw");

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.a) {
            imu.resetYaw();
        }
        // If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)
        if (gamepad1.left_bumper) {
            MDF.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            MDF.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }


}
