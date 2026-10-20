package org.firstinspires.ftc.teamcode;
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
    private Follower follower;

    private Localizer localizer;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(133.1455, 8.9993, 90);
    private final Pose path1 = poseFactory.of(68.3912, 125.3369, 180);
    private final Pose path1Control1 = poseFactory.of(128.3849, 114.5459, 0);
    private final Pose point2 = poseFactory.of(21.6278, 27.4301, 95.9389);
    private final Pose point2Control1 = poseFactory.of(11.4428, 122.5727, 0);
    private final Pose point3 = poseFactory.of(121.988, 28.4456, -179.4202);
    List<Path> paths = new ArrayList<Path>();
    int currentPath = 0;

    public Path path1() {
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.curve(path1, point2Control1, point2).reverseTangent();
    }

    public Path path3() {
        return Paths.line(point2, point3).reverseTangent();
    }


    @Override
    public void init() {
        paths.add(path1());
        paths.add(path2());
        paths.add(path3());
        Scheduler.reset();
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        localizer.setPose(start);
        follower = Constants.create(hardwareMap, localizer);
        follower.setPose(start);
    }
    @Override
    public void start() {
        schedule(follow(follower, path1()));
        //schedule(follow(follower, path2()));
        //schedule(follow(follower, path3()));
    }
    @Override
    public void loop() {
        follower.update();
        //localizer.update();
        Scheduler.execute();
        telemetry.addData("POSE: ",localizer.pose());
        telemetry.update();
        if(follower.completion() == 1 && currentPath < paths.toArray().length){
            currentPath++;
            schedule(follow(follower, paths.get(currentPath)));

        }
    }


}

