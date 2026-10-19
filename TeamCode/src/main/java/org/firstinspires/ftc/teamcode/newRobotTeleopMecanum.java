package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "Robot: USE THIS MECANUM", group = "Robot")
public class newRobotTeleopMecanum extends OpMode {

    private DcMotor transfer, shooter;
    mecanumDriveFunctions MDF = new mecanumDriveFunctions();
    intakeV3 intake = new intakeV3();
    targetLock TL = new targetLock();
    @Override
    public void init() {
        MDF.initMecanum(hardwareMap);
        intake.initIntake(hardwareMap);

        transfer = hardwareMap.get(DcMotor.class, "transfer");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        transfer.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);


    }

    public IMU imu;

    private enum transferState{
        MOTOR_ON(1.0),
        MOTOR_OFF(0.0);

        public final double power;

        transferState(double power) {
            this.power = power;
        }
    }

    private enum shooterState{
        MOTOR_ON(0.75),
        MOTOR_OFF(0.0);

        public final double power;

        shooterState(double power) {
            this.power = power;
        }
    }



    private newRobotTeleopMecanum.transferState transferStateM = newRobotTeleopMecanum.transferState.MOTOR_OFF;
    private newRobotTeleopMecanum.shooterState shooterStateM = newRobotTeleopMecanum.shooterState.MOTOR_OFF;
    public boolean IntakeDBounce, passerDBounce, shooterDBounce = false;




    @Override
    public void loop() {
        String GetSpeed = MDF.getSpeed().toString();
        telemetry.addLine("Press x to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");
        telemetry.addLine("Aim robot the robot direction as you and press x to reset yaw");
        telemetry.addLine(GetSpeed);
        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.right_bumper) {
            imu.resetYaw();
        }

        // If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)
        if (gamepad1.left_bumper) {
            MDF.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            MDF.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        if (gamepad1.b && !IntakeDBounce){
            intake.toggleState();
            IntakeDBounce = true;
        } else if (!gamepad1.b && IntakeDBounce) {
            IntakeDBounce = false;
        }


        if (gamepad1.x && !passerDBounce){
            if (transferStateM == transferState.MOTOR_OFF) {
                transferStateM = newRobotTeleopMecanum.transferState.MOTOR_ON;

            } else {
                transferStateM = newRobotTeleopMecanum.transferState.MOTOR_OFF;
            }

            transfer.setPower(transferStateM.power);
            passerDBounce = true;
        }
        else if (!gamepad1.x && passerDBounce){
            passerDBounce = false;
        }

        if (gamepad1.y && !shooterDBounce){
            if (shooterStateM == shooterState.MOTOR_OFF) {
                shooterStateM = newRobotTeleopMecanum.shooterState.MOTOR_ON;

            } else {
                shooterStateM = newRobotTeleopMecanum.shooterState.MOTOR_OFF;
            }

            shooter.setPower(shooterStateM.power);
            shooterDBounce = true;
        }
        else if (!gamepad1.y && shooterDBounce){
            shooterDBounce = false;
        }

    }


}
