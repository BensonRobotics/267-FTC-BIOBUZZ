package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
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
    public static Follower create(HardwareMap h, Localizer localizer) {
        return new Follower(localizer, new Mecanum(h, drivetrainConfig), new Foresight(foresightConfig));
    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fl");
        c.frontRightName.set("fr");
        c.backLeftName.set("rl");
        c.backRightName.set("rr");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-1.3008476617768054);
        c.yPodOffset.set(-0.0922295146101103);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.1385168390560008);
                Controller secondaryTranslationalForward = Controller.proportional(0.05117827079308604);
                Controller primaryTranslationalLateral = Controller.proportional(0.2051110987656637);
                Controller secondaryTranslationalLateral = Controller.proportional(0.07578307032441478);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01569193727076341));
                c.brake.set(Controller.proportionalFeedforward(0.0133381466801489));

                c.headingFeedback.set(Controller.proportional(3.064840270112769));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04234581749121029, 0.005189541610876401));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09164975949791256, 0.049551451173032744));
                c.quadraticBrakeCoefficients.set(Matrix.diag(4.1704127731626646E-4, 0.0016984112005397384));

                c.maxAchievableForwardVelocity.set(65.91000111561598);
                c.maxAchievableStrafeVelocity.set(59.74187279070985);
                c.naturalForwardDeceleration.set(37.57884595406801);
                c.naturalStrafeDeceleration.set(55.38471546685381);
            }
    );

}



