package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name="Tuneing test")
public class Tuneing_test extends LinearOpMode {

    // This variable will now appear in your web dashboard for live tuning
    public static double testPower = 0.5;


    @Override
    public void runOpMode() {
        telemetry = com.acmerobotics.dashboard.FtcDashboard.getInstance().getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Current Power", testPower);
            telemetry.update();
        }
    }
}