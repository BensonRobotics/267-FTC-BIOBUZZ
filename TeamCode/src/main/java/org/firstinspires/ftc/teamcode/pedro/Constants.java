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
        c.xPodOffset.set(0.3157779363196666);
        c.yPodOffset.set(1.7417095214363159);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.13757956250665712);
                Controller secondaryTranslationalForward = Controller.proportional(0.050831972152594204);
                Controller primaryTranslationalLateral = Controller.proportional(-23.346461437848525);
                Controller secondaryTranslationalLateral = Controller.proportional(-8.625893672346189);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01671984266536521));
                c.brake.set(Controller.proportionalFeedforward(0.014211866265560428));

                c.headingFeedback.set(Controller.proportional(3.425843938911667));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04708733771995951, 0.004469587517393355));

                c.linearBrakeCoefficients.set(Matrix.diag(0.058791950622661196, 0.051321369946270075));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0013814807366936577, 0.0016647013108102339));

                c.maxAchievableForwardVelocity.set(63.871166393866204);
                c.maxAchievableStrafeVelocity.set(57.68197377451063);
                c.naturalForwardDeceleration.set(39.39338058242505);
                c.naturalStrafeDeceleration.set(54.71820196262424);
            }
    );

}



