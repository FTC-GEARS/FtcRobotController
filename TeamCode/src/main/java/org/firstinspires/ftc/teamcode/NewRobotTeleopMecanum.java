package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Robot: MAIN MECANUM", group = "Robot")
@Config
public class NewRobotTeleopMecanum extends OpMode {

    private FtcDashboard dashboard;

    public static double maxIntake = 8;
    public static double maxPasser = 0.8;
    public static double maxPower = 1.0;
    public static double maxSpeed = 1.0;
    public static double shooterRPM = 1225;

    MecanumDriveFunctions MDF = new MecanumDriveFunctions();
    IntakeV3 intake = new IntakeV3();
    MainGuts mGuts = new MainGuts();

    @Override
    public void init() {
        MDF.initMecanum(hardwareMap, maxPower, maxSpeed);
        intake.initIntake(hardwareMap, maxIntake);
        mGuts.initGuts(hardwareMap, maxPasser, shooterRPM);
        dashboard = FtcDashboard.getInstance();
    }

    private boolean previousA, previousShoot, previousX, previousY, previousDU, previousDD, previousResetYaw = false;

    TelemetryPacket mainPacket = new TelemetryPacket();
    public void addTele(String caption, Object data){
        data = data.toString();
        telemetry.addData(caption, data);
        mainPacket.put(caption, data);

    }

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
            shooterRPM += 25;
        } else if (currentDD && !previousDD) {
            shooterRPM -= 25;
        }
        mGuts.updateShooter(shooterRPM);

        // Triggers

        if (currentY && !previousY) {
            mGuts.toggleShooter();
        }
        if (currentResetYaw && !previousResetYaw) {
            MDF.resetYaw();


        }
        if(gamepad2.left_bumper){
            intake.reversePasser();
            mGuts.reversePasser();
        }
        else {
            if (currentA && !previousA) {
                intake.toggleState();
            }
            if (currentShoot && !previousShoot) {
                mGuts.togglePasser();
            }
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
        addTele("Pinpoint X (in)", String.format("%.2f", pose.getX(DistanceUnit.INCH)));
        addTele("Pinpoint Y (in)", String.format("%.2f", pose.getY(DistanceUnit.INCH)));
        addTele("Pinpoint Heading (deg)", String.format("%.2f", pose.getHeading(AngleUnit.DEGREES)));
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

