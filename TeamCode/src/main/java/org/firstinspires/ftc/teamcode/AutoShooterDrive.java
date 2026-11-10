package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@Autonomous(name = "Auto: Shooter & Drive 50cm", group = "Autonomous")
public class AutoShooterDrive extends LinearOpMode {

    private final MecanumDriveFunctions MDF = new MecanumDriveFunctions();
    private final MainGuts mGuts = new MainGuts();
    private final IntakeV3 intake = new IntakeV3();

    private final double maxIntake = 0.8;
    private final double maxPasser = 0.8;
    private final double maxPower = 1.0;
    private final double maxSpeed = 1.0;
    private final double shooterRPM = 1300;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize drive base and robot mechanisms
        MDF.initMecanum(hardwareMap, maxPower, maxSpeed);
        mGuts.initGuts(hardwareMap, maxPasser, shooterRPM);
        intake.initIntake(hardwareMap, maxIntake);

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.update();

        // Wait for Driver Station Start button
        waitForStart();

        if (isStopRequested()) return;

        // 1. Turn on the shooter and intake
        if (mGuts.getShooterState() == MainGuts.ShooterState.MOTOR_OFF) {
            mGuts.toggleShooter();
        }
        intake.setIntakeState(IntakeV3.MotorState.MOTOR_ON);

        telemetry.addData("Status", "Shooter & Intake ON");
        telemetry.update();

        // 2. Wait 1 second
        sleep(3000);

        // 3. Turn on the passer
        mGuts.setPasserState(MainGuts.PasserState.MOTOR_ON);
        telemetry.addData("Status", "Shooter, Intake & Passer ON");
        telemetry.update();
        sleep(3000);

        // 4. Drive forward 50 cm using Pinpoint odometry distance tracking
        MDF.updatePinpoint();
        Pose2D startPose = MDF.getPose();
        double startX = startPose.getX(DistanceUnit.CM);
        double startY = startPose.getY(DistanceUnit.CM);

        double targetDistanceCm = 50.0;
        double drivePower = 0.4;
        long timeoutMs = 5000; // 5-second safety timeout
        long startTime = System.currentTimeMillis();

        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < timeoutMs) {
            MDF.updatePinpoint();
            Pose2D currentPose = MDF.getPose();

            double deltaX = currentPose.getX(DistanceUnit.CM) - startX;
            double deltaY = currentPose.getY(DistanceUnit.CM) - startY;
            double distanceTraveled = Math.hypot(deltaX, deltaY);

            if (distanceTraveled >= targetDistanceCm) {
                break;
            }

            // Drive forward (robot-relative)
            MDF.drive(drivePower, 0.0, 0.0);

            telemetry.addData("Status", "Driving Forward 50cm");
            telemetry.addData("Distance (cm)", String.format(Locale.US, "%.2f / %.2f", distanceTraveled, targetDistanceCm));
            telemetry.addData("Shooter Velocity", mGuts.getShooterVelocity());
            telemetry.update();
        }

        // Stop drive motors
        MDF.drive(0.0, 0.0, 0.0);

        // 5. Turn off shooter, passer, and intake
        intake.setIntakeState(IntakeV3.MotorState.MOTOR_OFF);
        mGuts.setPasserState(MainGuts.PasserState.MOTOR_OFF);
        if (mGuts.getShooterState() == MainGuts.ShooterState.MOTOR_ON) {
            mGuts.toggleShooter();
        }

        telemetry.addData("Status", "Autonomous Sequence Complete");
        telemetry.update();
    }
}

