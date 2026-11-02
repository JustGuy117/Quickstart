package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("f-l-drive");
        c.frontRightName.set("f-r-drive");
        c.backLeftName.set("b-l-drive");
        c.backRightName.set("b-r-drive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-4.346042542945682);
        c.yPodOffset.set(0.056735183310321);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.27402030628344776);
                Controller secondaryTranslationalForward = Controller.proportional(0.10124318121429965);
                Controller primaryTranslationalLateral = Controller.proportional(0.4110458357625505);
                Controller secondaryTranslationalLateral = Controller.proportional(0.15187045296724766);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01251562416035243));
                c.brake.set(Controller.proportionalFeedforward(0.010638280536299565));

                c.headingFeedback.set(Controller.proportional(4.335203254535803));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05316377562834893, 0.005022468013042011));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10015076374021235, 0.08262773543491315));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012977459022591449, 0.001298857024107519));

                c.maxAchievableForwardVelocity.set(74.01961291649626);
                c.maxAchievableStrafeVelocity.set(59.38067243722579);
                c.naturalForwardDeceleration.set(31.644966736971273);
                c.naturalStrafeDeceleration.set(84.32270425955525);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
