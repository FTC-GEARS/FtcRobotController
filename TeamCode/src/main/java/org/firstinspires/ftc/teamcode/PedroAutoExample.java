package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

/**
 * PedroAutoExample – a sample autonomous OpMode using Pedro Pathing.
 *
 * Route (all distances in inches, Pedro field frame: +X=forward, +Y=left):
 *   1. Drive forward 48 in
 *   2. Strafe right  24 in  (-Y direction)
 *   3. Cubic Bezier curve back to origin
 *   4. Hold the starting pose until time expires
 *
 * Customise FORWARD_DIST, STRAFE_DIST, and the waypoints to match your field.
 *
 * Key Pedro Pathing concepts shown here:
 *   - follower.follow(path)         – start following a path
 *   - follower.atParametricEnd()    – true once the robot reaches the path end
 *   - follower.hold(pose)           – lock onto a fixed pose
 *   - path heading interpolation    – .constant(rad) keeps a fixed heading,
 *                                     .tangent() faces the curve direction
 */
@Autonomous(name = "Pedro: Example Auto", group = "Pedro")
public class PedroAutoExample extends LinearOpMode {

    // ── Subsystems ────────────────────────────────────────────────────────────
    private Follower follower;
    private IntakeV3 intake = new IntakeV3();
    private MainGuts mGuts  = new MainGuts();

    // ── Tunable constants (edit to match your field) ─────────────────────────
    private static final double FORWARD_DIST = 48.0; // inches to drive forward
    private static final double STRAFE_DIST  = 24.0; // inches to strafe right
    private static final double MAX_PASSER   = 0.7;
    private static final double SHOOTER_RPM  = 1225;
    private static final double MAX_INTAKE   = 0.8;

    // ── Poses ─────────────────────────────────────────────────────────────────
    //   Pedro field frame: +X = forward, +Y = left, heading in radians (CCW+)
    private final Pose startPose   = Pose.zero();
    private final Pose forwardPose = new Pose(FORWARD_DIST, 0, 0);
    private final Pose cornerPose  = new Pose(FORWARD_DIST, -STRAFE_DIST, 0); // right = -Y

    // ── State machine ─────────────────────────────────────────────────────────
    private enum State {
        DRIVE_FORWARD,
        STRAFE_RIGHT,
        CURVE_HOME,
        HOLD_POSE,
        DONE
    }

    // -------------------------------------------------------------------------
    @Override
    public void runOpMode() {

        // ── Build Follower ────────────────────────────────────────────────────
        follower = new Follower(
                new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                new Mecanum(hardwareMap, Constants.drivetrainConfig),
                new Foresight(Constants.foresightConfig)
        );
        follower.setPose(startPose);

        // ── Subsystems ────────────────────────────────────────────────────────
        intake.initIntake(hardwareMap, MAX_INTAKE);
        mGuts.initGuts(hardwareMap, MAX_PASSER, SHOOTER_RPM);

        // ── Build paths up-front ──────────────────────────────────────────────
        // Segment 1 – straight line forward, face forward (heading = 0 rad)
        Path driveForward = line(startPose, forwardPose)
                .constant(0);  // maintain 0° heading throughout

        // Segment 2 – straight line to the right, face forward
        Path strafeRight = line(forwardPose, cornerPose)
                .constant(0);

        // Segment 3 – Bezier curve back home; robot faces the tangent of the path
        Path curveHome = curve(cornerPose,
                               new Pose(FORWARD_DIST / 2.0, -STRAFE_DIST / 2.0), // control point
                               startPose)
                .tangent();

        // ── Init telemetry ────────────────────────────────────────────────────
        telemetry.addData("Status", "Ready – Pedro Auto Example");
        telemetry.addData("Start Pose", startPose);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Lock in starting pose after the timer starts
        follower.setPose(startPose);

        // Kick off the first segment
        follower.follow(driveForward);
        State state = State.DRIVE_FORWARD;

        // ── Main loop ─────────────────────────────────────────────────────────
        while (opModeIsActive() && state != State.DONE) {

            follower.update();

            switch (state) {

                // ── Segment 1: drive forward ──────────────────────────────────
                case DRIVE_FORWARD:
                    if (follower.atParametricEnd()) {
                        // ---- robot action: spin intake briefly ----------------
                        intake.setIntakeState(IntakeV3.MotorState.MOTOR_ON);
                        sleep(500);
                        intake.setIntakeState(IntakeV3.MotorState.MOTOR_OFF);
                        // -------------------------------------------------------
                        follower.follow(strafeRight);
                        state = State.STRAFE_RIGHT;
                    }
                    break;

                // ── Segment 2: strafe right ───────────────────────────────────
                case STRAFE_RIGHT:
                    if (follower.atParametricEnd()) {
                        // ---- robot action: shoot once -------------------------
                        mGuts.toggleShooter();
                        sleep(1000);  // spin up
                        mGuts.togglePasser();
                        sleep(600);   // pass ball
                        mGuts.togglePasser();
                        mGuts.toggleShooter();
                        // -------------------------------------------------------
                        follower.follow(curveHome);
                        state = State.CURVE_HOME;
                    }
                    break;

                // ── Segment 3: curve back to start ────────────────────────────
                case CURVE_HOME:
                    if (follower.atParametricEnd()) {
                        follower.hold(startPose);
                        state = State.HOLD_POSE;
                    }
                    break;

                // ── Hold start pose until time expires ────────────────────────
                case HOLD_POSE:
                    // follower.update() (called above) keeps applying corrections.
                    // The while-loop condition handles the 30-second time limit.
                    break;
            }

            // ── Telemetry ─────────────────────────────────────────────────────
            Pose pose = follower.pose();
            telemetry.addData("State",          state);
            telemetry.addData("X (in)",         String.format("%.2f", pose.x()));
            telemetry.addData("Y (in)",         String.format("%.2f", pose.y()));
            telemetry.addData("Heading (deg)",  String.format("%.2f", Math.toDegrees(pose.heading())));
            telemetry.addData("At Path End",    follower.atParametricEnd());
            telemetry.addData("Path Complete %", String.format("%.1f", follower.completion() * 100));
            telemetry.update();
        }

        // ── Clean stop ────────────────────────────────────────────────────────
        follower.drivetrain.drive(new DrivePowers(0, 0, 0), true);
    }
}
