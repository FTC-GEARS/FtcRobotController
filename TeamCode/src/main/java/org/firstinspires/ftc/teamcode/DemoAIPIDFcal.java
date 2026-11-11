package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.Locale;

/**
 * Shooter velocity PIDF auto-calibration utility.
 *
 * HOW IT WORKS
 *   Stage 1 - Feedforward: runs the shooter at 100% power, measures the max velocity
 *             (ticks/sec) and computes F = 32767 / maxVelocity.
 *   Stage 2 - P sweep:     step-tests a ladder of P values (scaled from F) and scores each
 *             step response (rise time, overshoot, settling, steady-state error, ripple).
 *   Stage 3 - I sweep:     tries a few I values on top of the best P.
 *   Stage 4 - D sweep:     tries a few D values on top of the best P/I.
 *   FTC Dashboard: open http://192.168.43.1:8080/dash (Control Hub) and add a graph with
 *   "velocity" and "setpoint" to watch every step response live. Per-trial scores
 *   (cost, best_cost, P, I, D, ...) are plotted once each trial finishes.
 *   Results are shown on the Driver Station, written to the log, and saved to
 *   shooter_pidf_results.txt in the RC settings folder.
 *
 * SAFETY / USAGE
 *   - Clear the shooter path and take the balls/artifacts out. The flywheel will spin at
 *     full power for a few seconds and then repeatedly step between a low and high speed.
 *   - Use a fully charged battery. F depends on battery voltage, so tune near full charge.
 *   - INIT: D-pad up/down changes the target velocity (+/- 50 ticks/sec).
 *   - PLAY: starts calibration.   gamepad1 B (or STOP): aborts and turns the shooter off.
 *
 * NOTE: MainGuts passes "shooterRPM" straight into setVelocity(), which is in TICKS PER
 *       SECOND by default (not RPM). The target here is therefore also ticks/sec.
 */
@Utility(name = "Util: Demo AI PIDF Cal")
public class DemoAIPIDFcal extends LinearOpMode {

    // ------------------------------------------------------------------ CONFIG
    private static final String SHOOTER_NAME = "shooter";
    /** Only used to show an approximate RPM. Change to match your motor's encoder. */
    private static final double TICKS_PER_REV = 28.0;

    /** Target velocity (ticks/sec) to tune for. Adjustable in INIT with the D-pad. */
    private double targetVelocity = 1300;
    /** Each trial steps from LOW_FRACTION * target up to target. */
    private static final double LOW_FRACTION = 0.5;
    /** Target is clamped to this fraction of the measured max velocity (leave headroom). */
    private static final double MAX_TARGET_FRACTION = 0.9;

    private static final double TRIAL_WINDOW_S = 2.5;       // length of each step response
    private static final double SETTLE_BAND = 0.02;         // +/-2% of target counts as settled
    private static final double COAST_TIMEOUT_S = 6.0;
    private static final double PRESETTLE_TIMEOUT_S = 4.0;
    private static final double SPINUP_TIMEOUT_S = 8.0;
    private static final double MIN_BATTERY_V = 12.0;

    /** P ladder, as multiples of F. */
    private static final double[] P_MULTS = {0.05, 0.1, 0.2, 0.4, 0.8, 1.6, 3.2};
    /** I candidates, as multiples of the best P. */
    private static final double[] I_MULTS = {0.05, 0.1, 0.25, 0.5};
    /** D candidates, as multiples of the best P. */
    private static final double[] D_MULTS = {0.01, 0.03, 0.1};

    // ------------------------------------------------------------------ STATE
    private DcMotorEx shooter;
    private VoltageSensor battery;

    private double maxVelocity = 0;
    private double F = 0;
    private double lowVelocity, highVelocity;

    private Trial best = null;
    private int trialCount = 0;
    private String stage = "Idle";
    private String detail = "";
    private final StringBuilder log = new StringBuilder();
    private final ElapsedTime uiTimer = new ElapsedTime();

    // FTC Dashboard plotting
    private FtcDashboard dashboard;
    private final ElapsedTime plotTimer = new ElapsedTime();
    private static final double PLOT_PERIOD_MS = 20;
    private double currentSetpoint = 0;   // what we are commanding right now (ticks/s)
    private int phase = 0;                // 0 = coast / spin-up, 1 = pre-settle, 2 = step response

    private static class AbortException extends RuntimeException {
    }

    private static class Trial {
        double p, i, d, f;
        double riseTime, overshootPct, settleTime, sseTicks, rippleTicks, iae, cost;
        boolean unstable;
    }

    // ------------------------------------------------------------------ MAIN
    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        shooter.setDirection(DcMotor.Direction.REVERSE);           // same as MainGuts
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        battery = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next() : null;

        boolean prevUp = false, prevDown = false;
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_up && !prevUp) targetVelocity += 50;
            if (gamepad1.dpad_down && !prevDown) targetVelocity = Math.max(100, targetVelocity - 50);
            prevUp = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down;

            telemetry.addLine("SHOOTER PIDF AUTO-TUNE");
            telemetry.addLine("Clear the shooter path! Flywheel will spin at full power.");
            telemetry.addData("Target (ticks/s)", "%.0f  (~%.0f RPM)", targetVelocity, toRpm(targetVelocity));
            telemetry.addData("Battery (V)", battery != null ? String.format(Locale.US, "%.2f", battery.getVoltage()) : "n/a");
            if (battery != null && battery.getVoltage() < MIN_BATTERY_V) {
                telemetry.addLine("WARNING: battery low - results will not match a full battery.");
            }
            telemetry.addLine("D-pad up/down: adjust target | PLAY: start | gamepad1 B: abort");
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested()) return;

        double startVoltage = battery != null ? battery.getVoltage() : -1;

        try {
            // ---- Stage 1: feedforward ------------------------------------------------
            stage = "Stage 1/4: measuring max velocity";
            maxVelocity = measureMaxVelocity();
            if (maxVelocity < 50) {
                throw new IllegalStateException("Max velocity ~0. Check motor name, encoder cable and direction.");
            }
            F = 32767.0 / maxVelocity;

            double cap = MAX_TARGET_FRACTION * maxVelocity;
            boolean clamped = false;
            if (targetVelocity > cap) {
                targetVelocity = cap;
                clamped = true;
            }
            highVelocity = targetVelocity;
            lowVelocity = LOW_FRACTION * highVelocity;

            logLine(String.format(Locale.US, "Max velocity: %.0f ticks/s | F = %.4f%s",
                    maxVelocity, F, clamped ? " | target clamped to 90% of max" : ""));

            // Baseline: feedforward only
            stage = "Stage 1/4: feedforward-only baseline";
            tryCandidate("FF only", 0, 0, 0);

            // ---- Stage 2: P sweep ----------------------------------------------------
            stage = "Stage 2/4: sweeping P";
            int worse = 0;
            for (double m : P_MULTS) {
                double before = best.cost;
                Trial t = tryCandidate(String.format(Locale.US, "P=%.2fxF", m), m * F, 0, 0);
                if (t.cost > before * 1.5 || t.unstable) worse++; else worse = 0;
                if (worse >= 2) break;
            }
            // local refinement around best P
            double centerP = best.p;
            if (centerP > 0) {
                tryCandidate("P refine 0.7x", centerP * 0.7, 0, 0);
                tryCandidate("P refine 1.4x", centerP * 1.4, 0, 0);
            }

            // ---- Stage 3: I sweep ----------------------------------------------------
            stage = "Stage 3/4: sweeping I";
            double bestP = best.p;
            if (bestP > 0) {
                for (double m : I_MULTS) {
                    tryCandidate(String.format(Locale.US, "I=%.2fxP", m), bestP, m * bestP, 0);
                }
            }

            // ---- Stage 4: D sweep ----------------------------------------------------
            stage = "Stage 4/4: sweeping D";
            bestP = best.p;
            double bestI = best.i;
            if (bestP > 0) {
                for (double m : D_MULTS) {
                    tryCandidate(String.format(Locale.US, "D=%.2fxP", m), bestP, bestI, m * bestP);
                }
            }

            stage = "DONE";
        } catch (AbortException e) {
            stage = "ABORTED";
        } catch (IllegalStateException e) {
            stage = "ERROR";
            detail = e.getMessage();
        } finally {
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter.setPower(0);
        }

        // ---- Report ----------------------------------------------------------------
        String report = buildReport(startVoltage);
        com.qualcomm.robotcore.util.RobotLog.ii("DemoAIPIDFcal", report);
        if (best != null) {
            TelemetryPacket done = new TelemetryPacket();
            done.put("final_P", best.p);
            done.put("final_I", best.i);
            done.put("final_D", best.d);
            done.put("final_F", best.f);
            done.put("final_cost", best.cost);
            dashboard.sendTelemetryPacket(done);
        }
        try {
            File file = AppUtil.getInstance().getSettingsFile("shooter_pidf_results.txt");
            ReadWriteFile.writeFile(file, report);
        } catch (Exception ignored) {
        }

        while (opModeIsActive()) {
            telemetry.addData("Status", stage);
            if (!detail.isEmpty()) telemetry.addLine(detail);
            for (String line : report.split("\n")) telemetry.addLine(line);
            telemetry.update();
            sleep(250);
        }
    }

    // ------------------------------------------------------------------ STAGE HELPERS
    private double measureMaxVelocity() {
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setPower(1.0);
        phase = 0;
        currentSetpoint = 0;   // open-loop full power: no velocity setpoint

        ElapsedTime total = new ElapsedTime();
        ElapsedTime check = new ElapsedTime();
        double ref = Math.abs(shooter.getVelocity());

        // wait for the velocity to plateau
        while (total.seconds() < SPINUP_TIMEOUT_S) {
            checkAbort();
            showStatus(Math.abs(shooter.getVelocity()));
            if (check.seconds() >= 0.75) {
                double v = Math.abs(shooter.getVelocity());
                if (total.seconds() > 2.0 && Math.abs(v - ref) < 0.01 * Math.max(v, 1)) break;
                ref = v;
                check.reset();
            }
            sleep(10);
        }

        // average for 1 second at steady state
        double sum = 0;
        int n = 0;
        ElapsedTime avg = new ElapsedTime();
        while (avg.seconds() < 1.0) {
            checkAbort();
            sum += Math.abs(shooter.getVelocity());
            n++;
            showStatus(Math.abs(shooter.getVelocity()));
            sleep(10);
        }
        shooter.setPower(0);
        return n > 0 ? sum / n : 0;
    }

    /** Runs one trial, updates the best result, and returns the trial. */
    private Trial tryCandidate(String label, double p, double i, double d) {
        detail = label;
        Trial t = runTrial(p, i, d, F);
        trialCount++;
        logLine(String.format(Locale.US,
                "%-14s P=%.3f I=%.3f D=%.3f | rise %.2fs  OS %.1f%%  settle %.2fs  sse %.1f  ripple %.1f  cost %.3f%s",
                label, t.p, t.i, t.d, t.riseTime, t.overshootPct, t.settleTime,
                t.sseTicks, t.rippleTicks, t.cost, t.unstable ? "  (UNSTABLE)" : ""));
        if (best == null || t.cost < best.cost) best = t;
        sendTrialSummary(t);
        return t;
    }

    private Trial runTrial(double p, double i, double d, double f) {
        // 1) Coast down below the low setpoint (no reverse torque on the flywheel)
        phase = 0;
        currentSetpoint = 0;
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setPower(0);
        ElapsedTime t = new ElapsedTime();
        while (Math.abs(shooter.getVelocity()) > lowVelocity * 0.9 && t.seconds() < COAST_TIMEOUT_S) {
            checkAbort();
            showStatus(Math.abs(shooter.getVelocity()));
            sleep(10);
        }

        // 2) Apply coefficients and settle at the low setpoint
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(p, i, d, f, MotorControlAlgorithm.PIDF));
        shooter.setVelocity(lowVelocity);
        phase = 1;
        currentSetpoint = lowVelocity;

        t.reset();
        ElapsedTime inBand = new ElapsedTime();
        boolean wasInBand = false;
        while (t.seconds() < PRESETTLE_TIMEOUT_S) {
            checkAbort();
            double v = Math.abs(shooter.getVelocity());
            showStatus(v);
            boolean ok = Math.abs(v - lowVelocity) < 0.05 * lowVelocity;
            if (ok && !wasInBand) inBand.reset();
            wasInBand = ok;
            if (ok && inBand.seconds() > 0.3) break;
            sleep(10);
        }

        // 3) Step to the high setpoint and record the response
        ArrayList<Double> ts = new ArrayList<>();
        ArrayList<Double> vs = new ArrayList<>();
        ElapsedTime step = new ElapsedTime();
        shooter.setVelocity(highVelocity);
        phase = 2;
        currentSetpoint = highVelocity;
        while (step.seconds() < TRIAL_WINDOW_S) {
            checkAbort();
            double v = Math.abs(shooter.getVelocity());
            ts.add(step.seconds());
            vs.add(v);
            showStatus(v);
            sleep(5);
        }

        return analyze(p, i, d, f, ts, vs);
    }

    private Trial analyze(double p, double i, double d, double f,
                          ArrayList<Double> ts, ArrayList<Double> vs) {
        Trial r = new Trial();
        r.p = p;
        r.i = i;
        r.d = d;
        r.f = f;

        int n = vs.size();
        double stepSize = highVelocity - lowVelocity;
        double window = TRIAL_WINDOW_S;
        double v90 = lowVelocity + 0.9 * stepSize;
        double band = SETTLE_BAND * highVelocity;

        double peak = 0;
        double rise = window;
        double lastOut = 0;
        double iaeSum = 0;
        double tailSum = 0, tailSumSq = 0;
        int tailN = 0;

        for (int k = 0; k < n; k++) {
            double tk = ts.get(k);
            double vk = vs.get(k);
            peak = Math.max(peak, vk);
            if (rise == window && vk >= v90) rise = tk;
            if (Math.abs(vk - highVelocity) > band) lastOut = tk;
            if (k > 0) {
                double dt = tk - ts.get(k - 1);
                iaeSum += 0.5 * (Math.abs(highVelocity - vk) + Math.abs(highVelocity - vs.get(k - 1))) * dt;
            }
            if (tk >= window - 0.5) {
                tailSum += vk;
                tailSumSq += vk * vk;
                tailN++;
            }
        }

        double tailMean = tailN > 0 ? tailSum / tailN : 0;
        double tailVar = tailN > 0 ? Math.max(0, tailSumSq / tailN - tailMean * tailMean) : 0;

        r.riseTime = rise;
        double overshootFrac = Math.max(0, (peak - highVelocity) / stepSize);
        r.overshootPct = overshootFrac * 100.0;
        r.settleTime = lastOut;
        r.sseTicks = Math.abs(tailMean - highVelocity);
        r.rippleTicks = Math.sqrt(tailVar);
        r.iae = iaeSum / (stepSize * window);
        r.unstable = overshootFrac > 0.5 || (r.rippleTicks / highVelocity) > 0.10;

        double sseFrac = r.sseTicks / highVelocity;
        double rippleFrac = r.rippleTicks / highVelocity;
        double settleFrac = r.settleTime / window;
        r.cost = 1.0 * r.iae
                + 1.5 * overshootFrac
                + 10.0 * sseFrac
                + 10.0 * rippleFrac
                + 0.5 * settleFrac
                + (r.unstable ? 10.0 : 0.0);
        return r;
    }

    // ------------------------------------------------------------------ UTIL
    private void checkAbort() {
        if (isStopRequested() || gamepad1.b) throw new AbortException();
    }

    private double toRpm(double ticksPerSec) {
        return ticksPerSec / TICKS_PER_REV * 60.0;
    }

    private void logLine(String s) {
        log.append(s).append("\n");
    }

    /** Streams the live velocity/setpoint to FTC Dashboard (throttled). */
    private void plot(double currentVelocity) {
        if (plotTimer.milliseconds() < PLOT_PERIOD_MS) return;
        plotTimer.reset();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("velocity", currentVelocity);
        packet.put("setpoint", currentSetpoint);
        packet.put("phase", phase);
        packet.put("trial", trialCount + 1);
        if (phase == 2) {
            packet.put("band_hi", highVelocity * (1 + SETTLE_BAND));
            packet.put("band_lo", highVelocity * (1 - SETTLE_BAND));
        }
        dashboard.sendTelemetryPacket(packet);
    }

    /** One packet per finished trial: scores and coefficients, for the cost/trial graphs. */
    private void sendTrialSummary(Trial t) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("trial_done", trialCount);
        packet.put("cost", t.cost);
        packet.put("best_cost", best.cost);
        packet.put("P", t.p);
        packet.put("I", t.i);
        packet.put("D", t.d);
        packet.put("rise_s", t.riseTime);
        packet.put("overshoot_pct", t.overshootPct);
        packet.put("settle_s", t.settleTime);
        packet.put("sse_ticks", t.sseTicks);
        packet.put("ripple_ticks", t.rippleTicks);
        dashboard.sendTelemetryPacket(packet);
    }

    private void showStatus(double currentVelocity) {
        plot(currentVelocity);
        if (uiTimer.milliseconds() < 100) return;
        uiTimer.reset();
        telemetry.addData("Stage", stage);
        telemetry.addData("Testing", detail);
        telemetry.addData("Trials done", trialCount);
        telemetry.addData("Velocity (ticks/s)", "%.0f", currentVelocity);
        if (maxVelocity > 0) {
            telemetry.addData("Max vel / F", "%.0f / %.3f", maxVelocity, F);
            telemetry.addData("Target (low -> high)", "%.0f -> %.0f", lowVelocity, highVelocity);
        }
        if (best != null) {
            telemetry.addData("Best so far", "P=%.3f I=%.3f D=%.3f F=%.3f (cost %.3f)",
                    best.p, best.i, best.d, best.f, best.cost);
        }
        telemetry.addLine("gamepad1 B: abort");
        telemetry.update();
    }

    private String buildReport(double startVoltage) {
        StringBuilder sb = new StringBuilder();
        sb.append("=== Shooter PIDF Auto-Tune ===\n");
        sb.append(String.format(Locale.US, "Status: %s\n", stage));
        if (startVoltage > 0) sb.append(String.format(Locale.US, "Battery at start: %.2f V\n", startVoltage));
        if (maxVelocity > 0) {
            sb.append(String.format(Locale.US, "Max velocity: %.0f ticks/s (~%.0f RPM)\n", maxVelocity, toRpm(maxVelocity)));
            sb.append(String.format(Locale.US, "Tuned target: %.0f ticks/s (~%.0f RPM)\n", highVelocity, toRpm(highVelocity)));
        }
        if (best != null) {
            sb.append(String.format(Locale.US, "BEST: P=%.4f  I=%.4f  D=%.4f  F=%.4f\n", best.p, best.i, best.d, best.f));
            sb.append(String.format(Locale.US, "  rise %.2fs | overshoot %.1f%% | settle %.2fs | sse %.1f | ripple %.1f\n",
                    best.riseTime, best.overshootPct, best.settleTime, best.sseTicks, best.rippleTicks));
            sb.append("Paste into MainGuts.initGuts():\n");
            sb.append("  shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);\n");
            sb.append(String.format(Locale.US,
                    "  shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,\n      new PIDFCoefficients(%.4f, %.4f, %.4f, %.4f, MotorControlAlgorithm.PIDF));\n",
                    best.p, best.i, best.d, best.f));
        }
        sb.append("--- Trial log ---\n").append(log);
        return sb.toString();
    }
}
/**
 * band_hi: 1326.0
 * band_lo: 1274.0
 * phase: 2
 * setpoint: 1300.0
 * trial: 17
 * velocity: 1300.0
 * Best so far: P=54.362 I=0.000 D=0.000 F=12.134 (cost 0.373)
 * Max vel / F: 2700 / 12.134
 * Stage: Stage 4/4: sweeping D
 * Target (low -> high): 650 -> 1300
 * Testing: D=0.10xP
 * Trials done: 16
 * Velocity (ticks/s): 1300
 * D: 5.4362346770276835
 * I: 0.0
 * P: 54.36234677027683
 * best_cost: 0.3446671903917707
 * cost: 0.3446671903917707
 * overshoot_pct: 3.076923076923077
 * ripple_ticks: 8.190161309362882
 * rise_s: 0.421969084
 * settle_s: 0.511071801
 * sse_ticks: 4.262295081967295
 * trial_done: 17
 * final_D: 5.4362346770276835
 * final_F: 12.13445240407965
 * final_I: 0.0
 * final_P: 54.36234677027683
 * final_cost: 0.3446671903917707
 * Status: DONE
 *
 * D=0.10xP
 * === Shooter PIDF Auto-Tune ===
 * Status: DONE
 * Battery at start: 13.69 V
 * Max velocity: 2700 ticks/s (~5786 RPM)
 * Tuned target: 1300 ticks/s (~2786 RPM)
 * BEST: P=54.3623 I=0.0000 D=5.4362 F=12.1345
 * rise 0.42s | overshoot 3.1% | settle 0.51s | sse 4.3 | ripple 8.2
 * Paste into MainGuts.initGuts():
 * shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
 * shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
 * new PIDFCoefficients(54.3623, 0.0000, 5.4362, 12.1345, MotorControlAlgorithm.PIDF));
 * --- Trial log ---
 * Max velocity: 2700 ticks/s | F = 12.1345
 * FF only P=0.000 I=0.000 D=0.000 | rise 1.57s OS 3.1% settle 1.98s sse 8.0 ripple 11.1 cost 0.883
 * P=0.05xF P=0.607 I=0.000 D=0.000 | rise 1.68s OS 3.1% settle 2.07s sse 12.3 ripple 15.1 cost 0.993
 * P=0.10xF P=1.213 I=0.000 D=0.000 | rise 1.60s OS 0.0% settle 2.03s sse 12.0 ripple 11.7 cost 0.899
 * P=0.20xF P=2.427 I=0.000 D=0.000 | rise 1.60s OS 0.0% settle 2.02s sse 8.7 ripple 11.2 cost 0.859
 * P=0.40xF P=4.854 I=0.000 D=0.000 | rise 1.37s OS 3.1% settle 1.91s sse 1.7 ripple 12.3 cost 0.798
 * P=0.80xF P=9.708 I=0.000 D=0.000 | rise 1.07s OS 3.1% settle 1.49s sse 10.2 ripple 10.0 cost 0.716
 * P=1.60xF P=19.415 I=0.000 D=0.000 | rise 0.77s OS 6.2% settle 1.84s sse 14.8 ripple 8.8 cost 0.801
 * P=3.20xF P=38.830 I=0.000 D=0.000 | rise 0.50s OS 3.1% settle 0.69s sse 8.3 ripple 9.9 cost 0.444
 * P refine 0.7x P=27.181 I=0.000 D=0.000 | rise 0.58s OS 3.1% settle 0.84s sse 4.5 ripple 9.1 cost 0.447
 * P refine 1.4x P=54.362 I=0.000 D=0.000 | rise 0.43s OS 3.1% settle 0.53s sse 5.0 ripple 10.1 cost 0.373
 * I=0.05xP P=54.362 I=2.718 D=0.000 | rise 0.37s OS 18.5% settle 2.49s sse 31.8 ripple 10.5 cost 1.270
 * I=0.10xP P=54.362 I=5.436 D=0.000 | rise 0.38s OS 33.8% settle 1.77s sse 10.0 ripple 10.0 cost 1.208
 * I=0.25xP P=54.362 I=13.591 D=0.000 | rise 0.40s OS 49.2% settle 2.28s sse 37.0 ripple 27.3 cost 1.907
 * I=0.50xP P=54.362 I=27.181 D=0.000 | rise 0.39s OS 43.1% settle 1.68s sse 3.6 ripple 9.2 cost 1.250
 * D=0.01xP P=54.362 I=0.000 D=0.544 | rise 0.45s OS 3.1% settle 0.52s sse 8.5 ripple 9.9 cost 0.400
 * D=0.03xP P=54.362 I=0.000 D=1.631 | rise 0.44s OS 3.1% settle 0.58s sse 7.9 ripple 9.8 cost 0.405
 * D=0.10xP P=54.362 I=0.000 D=5.436 | rise 0.42s OS 3.1% settle 0.51s sse 4.3 ripple 8.2 cost 0.345
 *
 */