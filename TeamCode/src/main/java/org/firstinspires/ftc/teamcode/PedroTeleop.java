package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;

@TeleOp(name = "Robot: PEDRO MECANUM", group = "Robot")
@Config
public class PedroTeleop extends OpMode {

    private FtcDashboard dashboard;

    public static double maxIntake = 0.8;
    public static double maxPasser = 0.7;
    public static double maxPower = 1.0;
    public static double maxSpeed = 1.0;
    public static double shooterRPM = 1225;

    IntakeV3 intake = new IntakeV3();
    MainGuts mGuts = new MainGuts();
    private Follower follower;


    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        intake.initIntake(hardwareMap, maxIntake);
        mGuts.initGuts(hardwareMap, maxPasser, shooterRPM);
        dashboard = FtcDashboard.getInstance();
    }

    private boolean previousA, previousShoot, previousX, previousY, previousDU, previousDD, previousL1, previousResetYaw = false;

    TelemetryPacket mainPacket = new TelemetryPacket();
    public void addTele(String caption, Object data){
        data = data.toString();
        telemetry.addData(caption, data);
        mainPacket.put(caption, data);

    }

    @Override
    public void loop() {

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                follower.pose().heading()
        );
        ManualDrive.driveOrHold(follower, powers);

        follower.update();

        Pose robotPose = follower.pose(); // returns a Pose object



        // Button inputs
        boolean currentA = gamepad2.a;
        boolean currentShoot = gamepad2.b||gamepad1.b;
        boolean currentX = gamepad2.x;
        boolean currentY = gamepad2.y;
        boolean currentDU = gamepad2.dpad_up;
        boolean currentDD = gamepad2.dpad_down;
        boolean currentResetYaw = gamepad1.left_trigger_pressed && gamepad1.right_trigger_pressed;
        boolean currentL1 = gamepad2.left_bumper;

        if (currentDU && !previousDU) {
            shooterRPM += 10;
        } else if (currentDD && !previousDD) {
            shooterRPM -= 10;
        }
        mGuts.updateShooter(shooterRPM);

        // Triggers

        if (currentY && !previousY) {
            mGuts.toggleShooter();
        }

        if (currentResetYaw && !previousResetYaw) {
            Pose cornerPose = new Pose(robotPose.x(), robotPose.y(), Math.toRadians(0));

            follower.setPose(cornerPose);
        }

        if(!currentL1){
            if (currentA && !previousA) {
                intake.toggleState();
            }
            if (currentShoot && !previousShoot) {
                mGuts.togglePasser();
            }
        }

        if(currentL1){
            intake.reverseIntake(true);
            mGuts.reversePasser(true);
        }
        else if(previousL1){
            intake.reverseIntake(false);
            mGuts.reversePasser(false);
        }



        // Save state for next frame
        previousA = currentA;
        previousShoot = currentShoot;
        previousX = currentX;
        previousY = currentY;
        previousDU = currentDU;
        previousDD = currentDD;
        previousResetYaw = currentResetYaw;
        previousL1 = currentL1;

        addTele("Robot X", robotPose.x());
        addTele("Robot Y", robotPose.y());
        addTele("Robot Heading", Math.toDegrees(robotPose.heading()));
        addTele("Passer State", mGuts.getPasserState());
        addTele("Passer Power", mGuts.getPasserPower())  ;
        addTele("Shooter State", mGuts.getShooterState());
        addTele("Shooter Velocity", mGuts.getShooterVelocity());
        addTele("Target Shooter RPM", shooterRPM);
        addTele("--- Controls ---", "");
        addTele("X: Toggle Intake | O: Toggle Passer | tri: Toggle Shooter","");

        dashboard.sendTelemetryPacket(mainPacket);
    }
}

