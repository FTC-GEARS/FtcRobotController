package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name = "IntakeV2", group = "TeleOp")
public class IntakeV2 extends OpMode {

    public static PIDF PIDF = new PIDF(0,0,0,0);
    private MotorState currentState;
    private boolean previousGamepadA;
    private static DcMotorEx intake;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // State & input tracking variables
        MotorState currentState = MotorState.MOTOR_OFF;
        boolean previousGamepadA = false;

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.update();
    }


    // States representing the motor status
    enum MotorState {
        MOTOR_OFF(0.0),
        MOTOR_ON(1.0);

        final double power;

        MotorState(double power) {
            this.power = power;
        }
    }

    @Override
    public void loop() {
        // Hardware initialization

        double TPS = intake.getVelocity();
        double RPM = TPS * 60 / 140;

        if (currentState == MotorState.MOTOR_ON) {
            PIDF.setTargetPosition(200);
        }

        if (currentState == MotorState.MOTOR_OFF) {
            PIDF.setTargetPosition(0);
        }

        PIDF.calculate(RPM);




        boolean currentGamepadA = gamepad1.a;

        // Detect rising edge press (just pressed 'A')
        if (currentGamepadA && !previousGamepadA) {
            // Toggle state
            currentState = (currentState == MotorState.MOTOR_OFF)
                    ? MotorState.MOTOR_ON
                    : MotorState.MOTOR_OFF;
        }
        previousGamepadA = currentGamepadA;

        // Apply motor output based on the state

        // Telemetry feedback
        telemetry.addData("Controls", "[A] Toggle Motor");
        telemetry.addData("State", currentState);
        telemetry.addData("Motor Power", "%.2f", intake.getPower());
        telemetry.addData("Motor position", "%.2f", intake.getCurrentPosition());
        telemetry.update();
    }
}