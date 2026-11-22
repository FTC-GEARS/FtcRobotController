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
        c.xPodOffset.set(1.3290929418849196);
        c.yPodOffset.set(-3.4746287188192055);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.46102775394639073);
                Controller secondaryTranslationalForward = Controller.proportional(0.17033743619472574);
                Controller primaryTranslationalLateral = Controller.proportional(0.5741810681904994);
                Controller secondaryTranslationalLateral = Controller.proportional(0.21214456229568246);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012588125952186046));
                c.brake.set(Controller.proportionalFeedforward(0.010699907059358138));

                c.headingFeedback.set(Controller.proportional(6.659266632082902));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06520393820372129, 0.002683218099717315));

                c.linearBrakeCoefficients.set(Matrix.diag(0.1780622433276608, 0.08032863364228515));
                c.quadraticBrakeCoefficients.set(Matrix.diag(-7.171504781494878E-4, 8.4519871294016E-4));

                c.maxAchievableForwardVelocity.set(72.62855561279011);
                c.maxAchievableStrafeVelocity.set(59.23377492136245);
                c.naturalForwardDeceleration.set(58.86796635897419);
                c.naturalStrafeDeceleration.set(89.44671828089764);
            }
    );
}