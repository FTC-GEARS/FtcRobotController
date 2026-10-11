package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config // Enables live variable tuning via FTC Dashboard
@TeleOp(name = "Dashboard Mechanism Control", group = "TeleOp")
public class DashboardMechanismControl extends LinearOpMode {

    // Declare motor hardware objects
    private DcMotor intake;
    private DcMotor transfer;
    private DcMotor shooter;

    // Static power variables editable in FTC Dashboard (http://192.168.43.1:8080/dash)
    public static double INTAKE_POWER = 0.8;
    public static double TRANSFER_POWER = 0.5;
    public static double SHOOTER_POWER = 1.0;

    @Override
    public void runOpMode() {

        // Initialize motors from Hardware Map
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        // Optional: Set motor directions if needed (e.g., shooter or intake spin in reverse)
        // shooter.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized. Connect to FTC Dashboard to adjust powers.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Button A controls Intake
            if (gamepad1.a) {
                intake.setPower(INTAKE_POWER);
            } else {
                intake.setPower(0);
            }

            // Button B controls Transfer
            if (gamepad1.b) {
                transfer.setPower(TRANSFER_POWER);
            } else {
                transfer.setPower(0);
            }

            // Button X controls Shooter
            if (gamepad1.x) {
                shooter.setPower(SHOOTER_POWER);
            } else {
                shooter.setPower(0);
            }

            // Telemetry output to Driver Station & Dashboard
            telemetry.addData("Intake Power", gamepad1.a ? INTAKE_POWER : 0);
            telemetry.addData("Transfer Power", gamepad1.b ? TRANSFER_POWER : 0);
            telemetry.addData("Shooter Power", gamepad1.x ? SHOOTER_POWER : 0);
            telemetry.update();
        }
    }
}