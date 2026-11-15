package org.firstinspires.ftc.teamcode.tests;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import static com.pedropathing.ivy.Scheduler.schedule;
import static java.util.concurrent.locks.LockSupport.park;

import java.util.ArrayList;
import java.util.List;


@Autonomous
public class OdometryTest extends OpMode {

    private Localizer localizer;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(0, 0, 90);



    @Override
    public void init() {
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        localizer.setPose(start);
    }
    @Override
    public void start() {
    }
    @Override
    public void loop() {
        localizer.update();
        telemetry.addData("Position: ", localizer.pose());
    }


}

