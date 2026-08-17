package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Mecanum Encoder Auto", group="Linear Opmode")
public class MecanumEncoderAuto extends LinearOpMode {

    private DcMotor frontLeft, backLeft, frontRight, backRight;
    //  The equation that will allwo you to find out how many ticks per inch you need
    // Adjust this constant based on your specific motor and wheel math!
    static final double TICKS_PER_INCH = 45.3;

    @Override
    public void runOpMode() {
        // Initialize hardware
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Most Mecanum setups require you to reverse the right motors so they drive forward

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset encoders to zero and set default run mode
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Ready to start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // STEP 1: Drive straight FORWARD 24 inches at 50% max speed
            // For forward: all targets are positive
            encoderDrive(0.9, 24, 24, 24, 24);

            // STEP 1a: Drive straight BACK 24 inches at 50% speed
            encoderDriveDirection(0.9, -24, 0, 0);

            // STEP 2: Strafe perfectly RIGHT 12 inches at 40% max speed
            // Mecanum Strafe Right targets: FL(+), BL(-), FR(-), BR(+)
            encoderDrive(0.8, 12, -12, -12, 12);

            // STEP 2a: Strafe perfectly LEFT 12 inches at 40% speed
            encoderDriveDirection(0.4, 0, -12, 0);

            // STEP 3: Spin/Turn LEFT 90 degrees (approx 15 inches of wheel travel)
            // Turn Left targets: FL(-), BL(-), FR(+), BR(+)
            encoderDrive(0.8, -30, -30, 30, 30);

            // STEP 3a: Spin/Turn counter-clockwise (Right) 30 inches of wheel travel
            encoderDriveDirection(0.4, 0, 0, 30);

            // STEP 4: Do a donut
            // Turn Left targets: FL(-), BL(-), FR(+), BR(+)
            encoderDrive(0.9, -30, 0, 30, 0);

            // STEP 4a: Do a donut
            encoderDriveDirection(0.4, 0, -30, 30);






        }
    }

    /**
     * Drives the robot based on field movement vectors (Forward, Strafe, Turn) in inches.
     * * @param speed      Maximum speed limit (0.0 to 1.0)
     * @param yInches    Distance to travel Forward (positive) or Backward (negative)
     * @param xInches    Distance to travel Strafe Right (positive) or Left (negative)
     * @param turnInches Distance for wheels to travel to Spin Right (positive) or Left (negative)
     */
    public void encoderDriveDirection(double speed, double yInches, double xInches, double turnInches) {
        int newFlTarget, newBlTarget, newFrTarget, newBrTarget;

        if (opModeIsActive()) {
            // Apply Mecanum kinematics to the desired distances (with a 1.1 multiplier to counteract strafe slip)
            double flInches = yInches + (xInches * 1.1) + turnInches;
            double blInches = yInches - (xInches * 1.1) + turnInches;
            double frInches = yInches - (xInches * 1.1) - turnInches;
            double brInches = yInches + (xInches * 1.1) - turnInches;

            // Calculate absolute target tick positions from current positions
            newFlTarget = frontLeft.getCurrentPosition() + (int)(flInches * TICKS_PER_INCH);
            newBlTarget = backLeft.getCurrentPosition() + (int)(blInches * TICKS_PER_INCH);
            newFrTarget = frontRight.getCurrentPosition() + (int)(frInches * TICKS_PER_INCH);
            newBrTarget = backRight.getCurrentPosition() + (int)(brInches * TICKS_PER_INCH);

            // Set targets
            frontLeft.setTargetPosition(newFlTarget);
            backLeft.setTargetPosition(newBlTarget);
            frontRight.setTargetPosition(newFrTarget);
            backRight.setTargetPosition(newBrTarget);

            // Turn on RUN_TO_POSITION mode
            setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Start movement (RUN_TO_POSITION expects positive speed limits)
            frontLeft.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            // Wait until all motors reach their destination
            while (opModeIsActive() &&
                    (frontLeft.isBusy() || backLeft.isBusy() || frontRight.isBusy() || backRight.isBusy())) {
                telemetry.addData("Moving To", "Y:%.1f, X:%.1f, Turn:%.1f", yInches, xInches, turnInches);
                telemetry.addData("Current Ticks", "FL:%d FR:%d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);

            // Reset back to standard encoder monitoring
            setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Reusable method to drive specific distances using the internal encoders.
     */
    public void encoderDrive(double speed, double flInches, double blInches, double frInches, double brInches) {
        int newFlTarget, newBlTarget, newFrTarget, newBrTarget;

        if (opModeIsActive()) {
            // Calculate target tick positions relative to where the robot currently is
            newFlTarget = frontLeft.getCurrentPosition() + (int)(flInches * TICKS_PER_INCH);
            newBlTarget = backLeft.getCurrentPosition() + (int)(blInches * TICKS_PER_INCH);
            newFrTarget = frontRight.getCurrentPosition() + (int)(frInches * TICKS_PER_INCH);
            newBrTarget = backRight.getCurrentPosition() + (int)(brInches * TICKS_PER_INCH);

            // Set target positions to the individual motors
            frontLeft.setTargetPosition(newFlTarget);
            backLeft.setTargetPosition(newBlTarget);
            frontRight.setTargetPosition(newFrTarget);
            backRight.setTargetPosition(newBrTarget);

            // Turn on RUN_TO_POSITION mode
            setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Start movement. RUN_TO_POSITION only requires positive velocity limits.
            frontLeft.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            // Keep looping while all motors are actively trying to reach their targets
            while (opModeIsActive() &&
                    (frontLeft.isBusy() || backLeft.isBusy() || frontRight.isBusy() || backRight.isBusy())) {

                // Display tracking data to the driver station
                telemetry.addData("Targets", "FL: %d, FR: %d", newFlTarget, newFrTarget);
                telemetry.addData("Current", "FL: %d, FR: %d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion once the targets are met
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);

            // Reset back to standard encoder mode for the next action
            setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // Small helper to quickly change modes on all 4 drivetrain motors at once
    private void setDriveMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        backLeft.setMode(mode);
        frontRight.setMode(mode);
        backRight.setMode(mode);
    }
}
