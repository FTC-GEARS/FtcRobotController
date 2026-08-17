package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "IntakeV2", group = "TeleOp")
public class IntakeV2 extends OpMode {

    private MotorState currentState;
    private boolean previousGamepadA;
    private DcMotor intake = null;
    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // State & input tracking variables
        MotorState currentState = MotorState.MOTOR_OFF;
        boolean previousGamepadA = false;

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.update();
    }
    // States representing the motor status
    private enum MotorState {
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
        intake.setPower(currentState.power);

        // Telemetry feedback
        telemetry.addData("Controls", "[A] Toggle Motor");
        telemetry.addData("State", currentState);
        telemetry.addData("Motor Power", "%.2f", intake.getPower());
        telemetry.update();
    }
}