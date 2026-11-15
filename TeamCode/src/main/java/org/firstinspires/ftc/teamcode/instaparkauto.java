package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@Autonomous(name = "side instant park", group = "Autonomous")
public class instaparkauto extends LinearOpMode {

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
        MDF.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90));

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.update();

        // Wait for Driver Station Start button
        waitForStart();

        if (isStopRequested()) return;



        // 4. Drive backward using Pinpoint odometry distance tracking
        double targetDistanceInches = 6.0;
        double drivePower = 0.4;
        long timeoutMs = 6000; // 6-second safety timeout
        driveByDistance(0.0, drivePower, 0.0, targetDistanceInches, DistanceUnit.INCH, timeoutMs, "Driving backward 40in");



        telemetry.addData("Status", "Autonomous Sequence Complete");
        telemetry.update();
    }





    /**
     * Drives using field-relative controls until the target distance is reached or timeout occurs.
     *
     * @param forward           Forward/backward power
     * @param right             Right/left strafe power
     * @param rotate            Rotation power
     * @param targetDistance    Target distance to travel
     * @param distanceUnit      Unit of distance (e.g. DistanceUnit.CM or DistanceUnit.INCH)
     * @param timeoutMs         Safety timeout in milliseconds
     * @param statusDescription Status message to display in telemetry
     */
    private void driveByDistance(double forward, double right, double rotate,
                                 double targetDistance, DistanceUnit distanceUnit,
                                 long timeoutMs, String statusDescription) {
        MDF.updatePinpoint();
        Pose2D startPose = MDF.getPose();
        double startX = startPose.getX(distanceUnit);
        double startY = startPose.getY(distanceUnit);

        long startTime = System.currentTimeMillis();

        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < timeoutMs) {
            MDF.updatePinpoint();
            Pose2D currentPose = MDF.getPose();

            double deltaX = currentPose.getX(distanceUnit) - startX;
            double deltaY = currentPose.getY(distanceUnit) - startY;
            double distanceTraveled = Math.hypot(deltaX, deltaY);

            if (distanceTraveled >= targetDistance) {
                break;
            }

            MDF.driveFieldRelative(forward, right, rotate);

            telemetry.addData("Status", statusDescription);
            String unitLabel = (distanceUnit == DistanceUnit.INCH) ? "in" : "cm";
            telemetry.addData("Distance (" + unitLabel + ")",
                    String.format(Locale.US, "%.2f / %.2f", distanceTraveled, targetDistance));
            telemetry.addData("Shooter Velocity", mGuts.getShooterVelocity());
            telemetry.update();
        }




        // Stop drive motors
        MDF.drive(0.0, 0.0, 0.0);
    }
}

