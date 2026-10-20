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
        c.xPodOffset.set(0.3178151198259489);
        c.yPodOffset.set(0.21916224261907144);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.140537840606028);
                Controller secondaryTranslationalForward = Controller.proportional(0.051924976863665116);
                Controller primaryTranslationalLateral = Controller.proportional(0.17804710888814226);
                Controller secondaryTranslationalLateral = Controller.proportional(0.06578364922779883);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01563592812940421));
                c.brake.set(Controller.proportionalFeedforward(0.013290538909993578));

                c.headingFeedback.set(Controller.proportional(3.161005637061404));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.053482955652833196, 0.0030262409896271627));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06234123822433195, 0.07742977020686949));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012803674312989782, 0.0010178319380805866));

                c.maxAchievableForwardVelocity.set(65.9149673112665);
                c.maxAchievableStrafeVelocity.set(56.334047103624755);
                c.naturalForwardDeceleration.set(39.294789861674765);
                c.naturalStrafeDeceleration.set(57.15362596445149);
            }
    );

}



