package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * PedroTeleop – looping TeleOp that replaces MecanumDriveFunctions with
 * Pedro Pathing's Follower for field-relative driving and pose telemetry.
 *
 * Controls (same as NewRobotTeleopMecanum):
 *   Gamepad 1
 *     Left stick          – translate (field-relative by default)
 *     Right stick X       – rotate
 *     Left bumper (held)  – switch to robot-relative driving
 *     Both triggers       – reset pose to (0,0,0)
 *   Gamepad 2
 *     A                   – toggle intake  (hold LB to reverse)
 *     B / Gamepad1 B      – toggle passer  (hold LB to reverse)
 *     Y                   – toggle shooter
 *     D-Pad Up / Down     – shooter RPM +/- 10
 *     Left bumper (held)  – reverse intake + passer
 */
@TeleOp(name = "Robot: PEDRO TELEOP", group = "Robot")
@Config
public class PedroTeleop extends OpMode {

    // ── Tunable constants ─────────────────────────────────────────────────────
    public static double maxIntake   = 0.8;
    public static double maxPasser   = 0.7;
    //public static double maxPower    = 1.0;
    public static double maxDriveSpeed = 1.0;
    public static double shooterRPM  = 1225;

    // ── Subsystems ────────────────────────────────────────────────────────────
    private Follower      follower;
    private IntakeV3      intake  = new IntakeV3();
    private MainGuts      mGuts   = new MainGuts();
    private FtcDashboard  dashboard;

    // ── Button edge-detection state ───────────────────────────────────────────
    private boolean previousA, previousShoot, previousX, previousY,
                    previousDU, previousDD, previousL1, previousResetYaw;

    // ── Dashboard packet ──────────────────────────────────────────────────────
    private TelemetryPacket mainPacket = new TelemetryPacket();

    // -------------------------------------------------------------------------
    @Override
    public void init() {
        // Build the Pedro Pathing Follower using the tuned constants
        follower = new Follower(
                new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                new Mecanum(hardwareMap, Constants.drivetrainConfig),
                new Foresight(Constants.foresightConfig)
        );

        // Start at origin
        follower.setPose(Pose.zero());

        // Subsystems
        intake.initIntake(hardwareMap, maxIntake);
        mGuts.initGuts(hardwareMap, maxPasser, shooterRPM);

        dashboard = FtcDashboard.getInstance();

        telemetry.addData("Status", "Initialized – Pedro Pathing TeleOp");
        telemetry.update();
    }

    // -------------------------------------------------------------------------
    @Override
    public void loop() {
        // ── 1. Update the follower (localizer + any active path/hold) ─────────
        follower.update();

        // ── 2. Drive control ─────────────────────────────────────────────────
        double forward = -gamepad1.left_stick_y;   // push up  = positive
        double strafe  =  gamepad1.left_stick_x;   // push right = positive
        double rotate  =  gamepad1.right_stick_x;

        if (gamepad1.left_bumper) {
            // ── Robot-relative: pass joystick values directly ─────────────────
            follower.manual(forward * maxDriveSpeed, strafe * maxDriveSpeed, rotate * maxDriveSpeed);
        } else {
            // ── Field-relative: rotate input vector by robot heading ───────────
            double heading = follower.pose().heading();
            double fieldForward = forward * Math.cos(-heading) - strafe * Math.sin(-heading);
            double fieldStrafe  = forward * Math.sin(-heading) + strafe * Math.cos(-heading);
            follower.manual(fieldForward * maxDriveSpeed, fieldStrafe * maxDriveSpeed, rotate * maxDriveSpeed);
        }

        // ── 3. Button inputs (edge-detected) ──────────────────────────────────
        /*boolean currentA         = gamepad2.a;
        boolean currentShoot     = gamepad2.b || gamepad1.b;
        boolean currentX         = gamepad2.x;
        boolean currentY         = gamepad2.y;
        boolean currentDU        = gamepad2.dpad_up;
        boolean currentDD        = gamepad2.dpad_down;
        boolean currentResetYaw  = gamepad1.left_trigger_pressed && gamepad1.right_trigger_pressed;
        boolean currentL1        = gamepad2.left_bumper;
*/
        // Shooter RPM adjustment
        if (gamepad2.dpadUpWasPressed()) {
            shooterRPM += 10;
        } else if (gamepad2.dpadDownWasPressed()) {
            shooterRPM -= 10;
        }
        mGuts.updateShooter(shooterRPM);

        // Toggle shooter
        if (gamepad2.yWasPressed()) {
            mGuts.toggleShooter();
        }

        // Reset pose to (0,0,0) – both triggers together
        if (gamepad1.leftTriggerWasPressed() && gamepad1.rightTriggerWasPressed()) {
            follower.setPose(Pose.zero());
        }

        // Intake & passer (blocked while LB is held for reverse)
        if (!gamepad2.left_bumper) {
            if (gamepad2.aWasPressed()) {
                intake.toggleState();
            }
            if (gamepad2.bWasPressed() || gamepad1.bWasPressed()) {
                mGuts.togglePasser();
            }
        }

        // LB held → reverse intake + passer
        if (gamepad2.leftBumperWasPressed()) {
            intake.reverseIntake(true);
            mGuts.reversePasser(true);
        } else if (gamepad2.leftBumperWasReleased()) {
            intake.reverseIntake(false);
            mGuts.reversePasser(false);
        }

        // Save previous button states
        /*previousA        = currentA;
        previousShoot    = currentShoot;
        previousX        = currentX;
        previousY        = currentY;
        previousDU       = currentDU;
        previousDD       = currentDD;
        previousResetYaw = currentResetYaw;
        previousL1       = currentL1;
*/
        // ── 4. Telemetry ──────────────────────────────────────────────────────
        mainPacket = new TelemetryPacket();

        Pose pose = follower.pose();
        addTele("Pedro X (in)",          String.format("%.2f", pose.x()));
        addTele("Pedro Y (in)",          String.format("%.2f", pose.y()));
        addTele("Pedro Heading (deg)",   String.format("%.2f", Math.toDegrees(pose.heading())));
        addTele("Drive Mode",            gamepad1.left_bumper ? "Robot-Relative" : "Field-Relative");
        addTele("Passer State",          mGuts.getPasserState());
        addTele("Passer Power",          mGuts.getPasserPower());
        addTele("Shooter State",         mGuts.getShooterState());
        addTele("Shooter Velocity",      mGuts.getShooterVelocity());
        addTele("Target Shooter RPM",    shooterRPM);
        addTele("--- Controls ---",      "");
        addTele("X: Toggle Intake | O: Toggle Passer | tri: Toggle Shooter", "");

        dashboard.sendTelemetryPacket(mainPacket);
        telemetry.update();
    }

    // ── Helper: post to both Driver Station and FTC Dashboard ─────────────────
    private void addTele(String caption, Object data) {
        String value = data.toString();
        telemetry.addData(caption, value);
        mainPacket.put(caption, value);
    }
}
