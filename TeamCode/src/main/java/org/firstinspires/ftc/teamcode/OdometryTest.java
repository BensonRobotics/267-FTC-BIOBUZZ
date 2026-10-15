package org.firstinspires.ftc.teamcode;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;

import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static java.util.concurrent.locks.LockSupport.park;


@Autonomous
public class OdometryTest extends OpMode {
    private Follower follower;

    private Localizer localizer;
    private final PoseFactory poseFactory = PoseFactory.degrees();


    private final Pose start = poseFactory.of(0, 0, 90);
    private final Pose path1 = poseFactory.of(0, 25, 90);

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }


    @Override
    public void init() {
        Scheduler.reset();
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        localizer.setPose(start);
        follower = Constants.create(hardwareMap, localizer);
        follower.setPose(start);
    }
    @Override
    public void start() {
        schedule(follow(follower, path1()));
    }
    @Override
    public void loop() {
        follower.update();
        localizer.update();
        Scheduler.execute();
        telemetry.addData("POSE: ",localizer.pose());
        telemetry.update();
    }


}

