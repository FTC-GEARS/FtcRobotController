package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Auto: pedro ", group = "Autonomous")
public class AutoShooterDrivePedro extends OpMode {
    private Follower follower;
    private final MainGuts mGuts = new MainGuts();
    private final IntakeV3 intake = new IntakeV3();
    private final double maxIntake = 0.8;
    private final double maxPasser = 0.8;
    private final double shooterRPM = 1215;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses
    private final Pose startPose        = poseFactory.of(60, 9, 270);
    private final Pose scorePose        = poseFactory.of(60, 13, 270);
    private final Pose controlPose1     = poseFactory.of(24, 24, 270);
    private final Pose pickupPose       = poseFactory.of(9, 10, 180);
    private final Pose parkPose         = poseFactory.of(17, 94, 270);
    private final Pose oppositestartPose = poseFactory.of(65, 130, 270);
    private final Pose farparkPose      = poseFactory.of(17, 102, 270);

    // Path methods
    private Path startToScore() { return line(startPose, scorePose).linear(startPose, scorePose); }
    //private Path scoreToPickup() { return line(scorePose, pickupPose).linear(startPose, pickupPose); }
    //private Path pickupToScore() { return line(pickupPose, scorePose).linear(pickupPose, scorePose); }
    private Path park() { return curve(scorePose, controlPose1, parkPose).linear(scorePose, parkPose); }

    // -------------------------------------------------------------------------
    // Ivy Commands
    // -------------------------------------------------------------------------

    /** Spins the shooter up to shooterRPM using PIDF. Completes when within 5% of target. */
    public Command shooterOn() {
        return Command.build()
                .setStart(() -> {
                    if (mGuts.getShooterState() == MainGuts.ShooterState.MOTOR_OFF) {
                        mGuts.toggleShooter(); // → MOTOR_ON + shooter.setVelocity(shooterRPM)
                    }
                })
                .setExecute(() -> mGuts.updateShooter(shooterRPM))
                .setDone(() -> {
                    double current = mGuts.getShooterVelocity();
                    return Math.abs(current - shooterRPM) < shooterRPM * 0.05;
                })
                .setEnd(endCondition -> {
                    // Shooter stays on so the robot can fire while driving
                });
    }

    /** Turns the intake motor ON (same power level used in MainGuts / TeleOp). */
    public Command intakeOn() {
        return instant(() -> intake.setIntakeState(IntakeV3.MotorState.MOTOR_ON));
    }

    /** Turns the intake motor OFF. */
    public Command intakeOff() {
        return instant(() -> intake.setIntakeState(IntakeV3.MotorState.MOTOR_OFF));
    }

    /**
     * Turns the passer (transfer) motor ON, waits 2 second, then turns it OFF.
     * Uses the same togglePasser() logic as MainGuts TeleOp controls.
     */
    public Command passerOnThenOff() {
        return sequential(
                instant(() -> mGuts.togglePasser()),   // → PasserState.MOTOR_ON
                waitMs(2000),
                instant(() -> mGuts.togglePasser())    // → PasserState.MOTOR_OFF
        );
    }

    // -------------------------------------------------------------------------

    private Command autoRoutine() {
        return sequential(
                shooterOn(),
                intakeOn(),
                follow(follower, startToScore()),
                passerOnThenOff(),
                follow(follower, park())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        mGuts.initGuts(hardwareMap, maxPasser, shooterRPM);
        intake.initIntake(hardwareMap, maxIntake);
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}
