/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

// Commented out REV IMU imports
// import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
// import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */public class MecanumDriveFunctions {
    // This declares the four motors needed
    private DcMotor frontLeft, backLeft, frontRight, backRight;

    // Commented out IMU needed to get the current direction the robot is facing
    // IMU imu;

    // GoBILDA Pinpoint Odometry Computer driver
    private GoBildaPinpointDriver pinpoint;

    private double maxPower;
    private double maxSpeed;


    public void initMecanum(HardwareMap hardwareMap, double maxPower, double maxSpeed) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        this.maxPower = maxPower;
        this.maxSpeed = maxSpeed;

        // Most Mecanum setups require you to reverse the right motors so they drive forward

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);



        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /* Commented out REV IMU initialization
        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        */

        // GoBILDA Pinpoint initialization
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Set odometry pod offsets in MM (X offset, Y offset) relative to tracking center
        // X pod offset: sideways distance to X (forward) pod (left is positive, right is negative)
        // Y pod offset: forward distance to Y (strafe) pod (forward is positive, backward is negative)
        pinpoint.setOffsets(70, 110, DistanceUnit.MM);

        // Set odometry pod resolution (e.g. goBILDA 4-bar pod or swingarm pod)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set odometry pod directions
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Reset position to (0,0,0) and recalibrate IMU on pinpoint
        //pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

    }

    public void updatePinpoint() {
        if (pinpoint != null) {
            pinpoint.update();
        }
    }

    // This routine drives the robot field relative
    public void driveFieldRelative(double forward, double right, double rotate) {
        // Update pinpoint data
        updatePinpoint();

        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        double currentHeading = (pinpoint != null) ? pinpoint.getPosition().getHeading(AngleUnit.RADIANS) : 0.0;
        theta = AngleUnit.normalizeRadians(theta - currentHeading);

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Counteract Mecanum strafe friction loss (strafe requires slightly higher effort)
        newRight *= 1.1;

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!

    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;



        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all the motors in proportion to what they should
        // be and not get clipped
        //THIS IS WRONG!!!!!!!!!
        double maxPower = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeft.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRight.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeft.setPower(maxSpeed * (backLeftPower / maxPower));
        backRight.setPower(maxSpeed * (backRightPower / maxPower));

    }

    // Inside mecanumDriveFunctions.java
    public void resetYaw() {
        /* Commented out REV IMU reset
        if (imu != null) {
            imu.resetYaw();
        }
        */
        if (pinpoint != null) {
            pinpoint.resetPosAndIMU();
        }
    }

    public GoBildaPinpointDriver getPinpoint() {
        return pinpoint;
    }

    public Pose2D getPose() {
        if (pinpoint != null) {
            return pinpoint.getPosition();
        }
        return new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);
    }

    public String[] getTelemtry(){
        Pose2D pose = getPose();
        return new String[]{
                "X (in): " + String.format("%.2f", pose.getX(DistanceUnit.INCH)),
                "Y (in): " + String.format("%.2f", pose.getY(DistanceUnit.INCH)),
                "Heading (deg): " + String.format("%.2f", pose.getHeading(AngleUnit.DEGREES)),
                "FL:" + Double.toString(frontLeft.getPower()),
                "FR:" + Double.toString(frontRight.getPower()),
                "BL:" + Double.toString(backLeft.getPower()),
                "BR:" + Double.toString(backRight.getPower())
        };
    }
}

