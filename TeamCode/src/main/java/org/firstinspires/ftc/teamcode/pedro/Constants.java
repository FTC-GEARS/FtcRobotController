package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);

        //c.manualBrakeMode.set(true)
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(1.5172216460460752);
        c.yPodOffset.set(-4.481688672163355);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.4713378607639361);
                Controller secondaryTranslationalForward = Controller.proportional(0.17414674517268078);
                Controller primaryTranslationalLateral = Controller.proportional(0.7302824118875528);
                Controller secondaryTranslationalLateral = Controller.proportional(0.2698198376870198);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013494715874257969));
                c.brake.set(Controller.proportionalFeedforward(0.011470508493119274));

                c.headingFeedback.set(Controller.proportional(6.942838600582667));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06845308637693191, 0.0016033601562689948));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09211829775094392, 0.08102794542596561));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014147809246104722, 0.0013382310061998742));

                c.maxAchievableForwardVelocity.set(78.67766619428278);
                c.maxAchievableStrafeVelocity.set(59.178889443640216);
                c.naturalForwardDeceleration.set(60.8307260148543);
                c.naturalStrafeDeceleration.set(95.12314569018896);
            }
    );
}