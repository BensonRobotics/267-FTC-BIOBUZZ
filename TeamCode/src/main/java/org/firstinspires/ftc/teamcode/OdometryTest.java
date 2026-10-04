package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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



@TeleOp
public class OdometryTest extends LinearOpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(9.5405, 132.5285, 90);
    private final Pose path1 = poseFactory.of(112.8546, 19.2406, 180);
    private final Pose path1Control1 = poseFactory.of(20.2597, 102.7174, 0);
    private final Pose path1Control2 = poseFactory.of(141.7384, 140.1312, 0);
    private final Pose path1Control3 = poseFactory.of(123.7931, 72.3658, 0);
    private final Pose path1Control4 = poseFactory.of(130.7369, 116.3096, 0);

    public Path path1() {
        return curve(start, path1Control1, path1Control2, path1Control3, path1Control4, path1).linear(start, path1);
    }


    @Override
    public void runOpMode(){
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        assert follower != null;
        follower.setPose(start);
        waitForStart();
        schedule(follow(follower, path1()));
        while(opModeIsActive()){
            follower.update();
            Scheduler.execute();
        }
    }
}

