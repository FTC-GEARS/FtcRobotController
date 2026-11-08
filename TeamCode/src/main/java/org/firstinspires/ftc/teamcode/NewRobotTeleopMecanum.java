package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Robot: MAIN MECANUM", group = "Robot")
public class NewRobotTeleopMecanum extends OpMode {

    public double maxIntake = 0.8;
    public double maxPasser = 0.8;
    public double maxPower = 1.0;
    public double maxSpeed = 1.0;
    public double shooterRPM = 1300;

    MecanumDriveFunctions MDF = new MecanumDriveFunctions();
    IntakeV3 intake = new IntakeV3();
    MainGuts mGuts = new MainGuts();

    @Override
    public void init() {
        MDF.initMecanum(hardwareMap, maxPower, maxSpeed);
        intake.initIntake(hardwareMap, maxIntake);
        mGuts.initGuts(hardwareMap, maxPasser, shooterRPM);
    }

    private boolean previousA, previousShoot, previousX, previousY, previousDU, previousDD, previousResetYaw = false;

    @Override
    public void loop() {
        // Update Pinpoint odometry computer
        MDF.updatePinpoint();

        // Drive control
        if (gamepad1.left_bumper) {
            MDF.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            MDF.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        // Button inputs
        boolean currentA = gamepad2.a;
        boolean currentShoot = gamepad2.b||gamepad1.b;
        boolean currentX = gamepad2.x;
        boolean currentY = gamepad2.y;
        boolean currentDU = gamepad2.dpad_up;
        boolean currentDD = gamepad2.dpad_down;
        boolean currentResetYaw = gamepad1.left_trigger_pressed && gamepad1.right_trigger_pressed;

        if (currentDU && !previousDU) {
            shooterRPM += 50;
        } else if (currentDD && !previousDD) {
            shooterRPM -= 50;
        }
        mGuts.updateShooter(shooterRPM);

        // Triggers
        if (currentA && !previousA) {
            intake.toggleState();
        }
        if (currentShoot && !previousShoot) {
            mGuts.togglePasser();
        }
        if (currentY && !previousY) {
            mGuts.toggleShooter();
        }
        if (currentResetYaw && !previousResetYaw) {
            MDF.resetYaw();
        }


        // Save state for next frame
        previousA = currentA;
        previousShoot = currentShoot;
        previousX = currentX;
        previousY = currentY;
        previousDU = currentDU;
        previousDD = currentDD;
        previousResetYaw = currentResetYaw;

        // Telemetry on Driver Station
        Pose2D pose = MDF.getPose();
        telemetry.addData("Pinpoint X (in)", String.format("%.2f", pose.getX(DistanceUnit.INCH)));
        telemetry.addData("Pinpoint Y (in)", String.format("%.2f", pose.getY(DistanceUnit.INCH)));
        telemetry.addData("Pinpoint Heading (deg)", String.format("%.2f", pose.getHeading(AngleUnit.DEGREES)));
        telemetry.addData("Passer State", mGuts.getPasserState());
        telemetry.addData("Passer Power", mGuts.getPasserPower());
        telemetry.addData("Shooter State", mGuts.getShooterState());
        telemetry.addData("Shooter Velocity", mGuts.getShooterVelocity());
        telemetry.addData("Target Shooter RPM", shooterRPM);
        telemetry.addLine("--- Controls ---");
        telemetry.addLine("X: Toggle Intake | O: Toggle Passer | tri: Toggle Shooter");
    }
}

