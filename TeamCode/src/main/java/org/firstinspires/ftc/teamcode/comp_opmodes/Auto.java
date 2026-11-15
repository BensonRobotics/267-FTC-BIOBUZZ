package org.firstinspires.ftc.teamcode.comp_opmodes;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.obselete.LauncherProto.lerp;

import android.os.Environment;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoMode;
import org.firstinspires.ftc.teamcode.util.Configuration;
import org.firstinspires.ftc.teamcode.util.MenuHandler;
import org.firstinspires.ftc.teamcode.util.MenuOptionExport;
import org.firstinspires.ftc.teamcode.util.StartingPos;
import org.firstinspires.ftc.teamcode.util.TurretState;

import java.io.File;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

@Autonomous(name = "Auto", group = "Comp", preselectTeleOp = "TeleOp")
public class Auto extends LinearOpMode {
    DcMotor intake;
    DcMotor feedMotor;

    DcMotorEx pollenTurretFlywheel;
    Servo pollenTurretHood;
    DcMotorEx pollenTurretDrive;

    boolean intakeOn;
    boolean launcherOn;

    //TODO: tune
    double intakeSpeed = 1;
    double feedSpeed = -0.9;

    //Hood settings
    //TODO: tune
    double servoHoodDefault = 0.5;
    double hoodMax = 0.6;
    double hoodMin = 0.2;

    double maxTurretRot = 750;
    double minTurretRot = -975;

    //Launcher settings
    double maxDistance = 1.2;

    //Speeds are in TPM (Ticks Per Minute) counts
    //TODO: tune
    double launcherMinSpeed = -2800;
    double launcherMaxSpeed = -1000;

    double launcherVelocityTolerance = 100;
    double launcherVelocityTarget = 0;

    Limelight3A limelight;

    //PID constants
    //TODO: tune
    final double p = 0.01d;
    final double i = 0.0d;
    final double d = 0.001d;

    //PID controller for pollen
    PIDController pollenPID;

    //Time to pause at the end of every path
    private final int pauseTime = 1000;
    private long timePaused;
    boolean paused = false;
    boolean waitingForNewPath = false;

    private Vector2D center = new Vector2D(70, 70);
    private TurretState pollenDriveState = TurretState.UNDEFINED;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    //Paths

    //Blue Starting Left of Park (blp)
    private final Pose blpstart = poseFactory.of(132.8953, 8.5941, 180);
    private final Pose blppath1 = poseFactory.of(84.8466, 21.7318, 90);
    private final Pose blppath1Control1 = poseFactory.of(106.6546, 27.3959, 0);
    private final Pose blppath2Start = poseFactory.of(84.8466, 21.7318, 90);
    private final Pose blppath2 = poseFactory.of(131.0618, 26.1769, 180);

    public Path blppath1() {
        return Paths.curve(blpstart, blppath1Control1, blppath1).linear(blpstart, blppath1);
    }

    public Path blppath2() {
        return Paths.line(blppath2Start, blppath2).linear(blppath2Start, blppath2);
    }

    public Path[] blpPaths = new Path[]{blppath1(), blppath2()};
    //Blue Starting Right Of Park (brp)
    private final Pose brpstart = poseFactory.of(132.3284, 57.0892, 180);
    private final Pose brppath1 = poseFactory.of(87.0356, 113.4685, -90);
    private final Pose brppath1Control1 = poseFactory.of(114.1536, 118.2692, 0);
    private final Pose brppath2Start = poseFactory.of(87.0356, 113.4685, -90);
    private final Pose brppath2 = poseFactory.of(129.9737, 46.2518, 0);
    private final Pose brppath2Control1 = poseFactory.of(118.3614, 116.9592, 0);

    public Path brppath1() {
        return Paths.curve(brpstart, brppath1Control1, brppath1).linear(brpstart, brppath1);
    }

    public Path brppath2() {
        return Paths.curve(brppath2Start, brppath2Control1, brppath2).linear(brppath2Start, brppath2);
    }
    public Path[] brpPaths = new Path[]{brppath1(), brppath2()};


    //Red Starting Left Of Park (rlp)
    private final Pose rlpstart = poseFactory.of(8.6172, 130.0891, 0);
    private final Pose rlppath1 = poseFactory.of(57.7866, 119.006, -90);
    private final Pose rlppath1Control1 = poseFactory.of(29.969, 114.5411, 0);
    private final Pose rlppath2Start = poseFactory.of(57.7866, 119.006, -90);
    private final Pose rlppath2 = poseFactory.of(10.5971, 118.2433, 0);

    public Path rlppath1() {
        return Paths.curve(rlpstart, rlppath1Control1, rlppath1).linear(rlpstart, rlppath1);
    }

    public Path rlppath2() {
        return Paths.line(rlppath2Start, rlppath2).linear(rlppath2Start, rlppath2);
    }
    public Path[] rlpPaths = new Path[]{rlppath1(), rlppath2()};


    //Red Starting Right Of Park (rrp)
    private final Pose rrpstart = poseFactory.of(8.514, 132.5668, 0);
    private final Pose rrppath1 = poseFactory.of(59.6287, 17.9496, 90);
    private final Pose rrppath1Control1 = poseFactory.of(25.5091, 44.1232, 0);
    private final Pose rrppath2Start = poseFactory.of(59.6287, 17.9496, 90);
    private final Pose rrppath2 = poseFactory.of(12.3748, 95.0798, 0);
    private final Pose rrppath2Control1 = poseFactory.of(10.8041, 51.7879, 0);

    public Path rrppath1() {
        return Paths.curve(rrpstart, rrppath1Control1, rrppath1).linear(rrpstart, rrppath1);
    }

    public Path rrppath2() {
        return Paths.curve(rrppath2Start, rrppath2Control1, rrppath2).linear(rrppath2Start, rrppath2);
    }

    public Path[] rrpPaths = new Path[]{rrppath1(), rrppath2()};


    @Override
    public void runOpMode() {
        //Initialize pollen PID controller
        pollenPID = new PIDController(p, i, d);

        //Initialize intake system
        intake = hardwareMap.get(DcMotor.class, "intake_motor");
        feedMotor = hardwareMap.get(DcMotor.class, "feed_motor");

        //Initialize pollen turret system
        pollenTurretFlywheel = hardwareMap.get(DcMotorEx.class, "pollen_launcher");
        pollenTurretHood = hardwareMap.get(Servo.class, "pollen_hood");
        pollenTurretDrive = hardwareMap.get(DcMotorEx.class, "pollen_turret_drive");

        //Initialize limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        //Create menu handler
        MenuHandler menuHandler = new MenuHandler(this.telemetry);
        boolean configurationCompleted = false;

        //Handle menu
        while (opModeInInit()) {
            //Move menu selector
            if (gamepad1.dpadDownWasPressed() || gamepad1.left_stick_y < -0.5) {
                menuHandler.ChangeSelection(1);
            } else if (gamepad1.dpadUpWasPressed() || gamepad1.left_stick_y > 0.5) {
                menuHandler.ChangeSelection(-1);
            } else if (gamepad1.dpadRightWasPressed() || gamepad1.leftStickButtonWasPressed()) {
                menuHandler.EditCurrentSelection();
            }

            //Finish menu and export
            if (gamepad1.startWasPressed()) {
                configurationCompleted = true;
                MenuOptionExport[] menuExports = menuHandler.ExportSelections();

                Configuration.alliance = Alliance.valueOf(menuExports[0].selection);
                if (Objects.equals(menuExports[1].selection, "Left Of Park")) {
                    Configuration.startingPos = StartingPos.LeftOfPark;
                } else {
                    Configuration.startingPos = StartingPos.RightOfPark;
                }

                if (Objects.equals(menuExports[2].selection, "Just Shoot")) {
                    Configuration.autoMode = AutoMode.JustShoot;
                } else if (Objects.equals(menuExports[2].selection, "Shoot Then Park")) {
                    Configuration.autoMode = AutoMode.ShootThenPark;
                }
                else{
                    Configuration.autoMode = AutoMode.Disabled;
                }
                menuHandler.LockOptions();
            }
            menuHandler.Redraw();
            telemetry.update();
        }
        if (!configurationCompleted) {
            telemetry.addLine("ERR: Configuration not completed. Please rerun the opmode and configure correctly.");
            telemetry.update();
            requestOpModeStop();
        }
        pollenTurretDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pollenTurretFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Scheduler.reset();

        Pose start = null;
        Path[] paths = null;
        int currentPathIdx = 0;

        if (Configuration.alliance == Alliance.Blue && Configuration.startingPos == StartingPos.LeftOfPark) {
            start = blpstart;
            paths = blpPaths;
        } else if (Configuration.alliance == Alliance.Blue && Configuration.startingPos == StartingPos.RightOfPark) {
            start = brpstart;
            paths = brpPaths;
        } else if (Configuration.alliance == Alliance.Red && Configuration.startingPos == StartingPos.RightOfPark) {
            start = rrpstart;
            paths = rrpPaths;
        } else /*if(Configuration.alliance == Alliance.Red && Configuration.startingPos == StartingPos.LeftOfPark) */ {
            start = rlpstart;
            paths = rlpPaths;
        }
        Localizer localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        localizer.setPose(start);
        Follower follower = Constants.create(hardwareMap, localizer);
        follower.setPose(start);


        if (Configuration.alliance == Alliance.Blue) {
            limelight.pipelineSwitch(0);
        } else if (Configuration.alliance == Alliance.Red) {
            limelight.pipelineSwitch(1);
        }

        waitForStart();
        pollenTurretDrive.setTargetPosition(0);
        pollenTurretDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pollenTurretDrive.setPower(1);
        pollenTurretFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Predefine pollen hood position
        pollenTurretHood.setPosition(0.5);

        if(Configuration.autoMode == AutoMode.Disabled){
            requestOpModeStop();
        }
        schedule(follow(follower, paths[0]));

        //Main event loop
        while (opModeIsActive()) {

            follower.update();
            Scheduler.execute();

            time = System.nanoTime();

            //Fetch limelight
            LLResult result = limelight.getLatestResult();

            //Check for limelight disconnect and warn user
            if (!limelight.isConnected()) {
                telemetry.addLine("WARN: Limelight disconnected. Robot reboot recommended. Check wiring.");
            }
            List<LLResultTypes.FiducialResult> fiducials =
                    (result != null) ? result.getFiducialResults() : Collections.emptyList();

            //Init variables for closest tag
            LLResultTypes.FiducialResult closestTag = null;
            double closestTagDistance = Double.POSITIVE_INFINITY;

            //Find closest april tag
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                Pose3D robotPose = fiducial.getRobotPoseTargetSpace();



                double distance = -robotPose.getPosition().z;
                if (distance < closestTagDistance) {
                    closestTagDistance = distance;
                    closestTag = fiducial;
                }
            }

            //Check for any tags
            if (closestTag == null) {
                launcherVelocityTarget = 0;
                Pose pose = localizer.pose();
                double bearing = Math.toDegrees(Math.atan2(72 - pose.y(), 72 - pose.x()));
                double rel = AngleUnit.normalizeDegrees(bearing - Math.toDegrees(pose.heading()));
                pollenTurretDrive.setTargetPosition((int) Math.round(clip(rel * (537.7 / 360) * 3.75, minTurretRot, maxTurretRot)));
                pollenPID.reset();
            } else {
                //Rough estimates for flywheel power & hood position
                pollenTurretHood.setPosition(lerp(hoodMin, hoodMax, closestTagDistance / maxDistance));

                //Toggle launcher hardware
                if (launcherOn) {
                    launcherVelocityTarget = lerp(launcherMinSpeed, launcherMaxSpeed, closestTagDistance / maxDistance);

                    //Only enable feed when pollen flywheel is up to speed
                    if (Math.abs(pollenTurretFlywheel.getVelocity() - launcherVelocityTarget) < launcherVelocityTolerance) {
                        feedMotor.setPower(feedSpeed);
                    } else {
                        feedMotor.setPower(0);
                    }
                } else {
                    launcherVelocityTarget = 0;
                    feedMotor.setPower(0);
                }

                //PID input for targeting
                pollenPID.setSetPoint(0.0);
                double correction = pollenPID.calculate(closestTag.getTargetXDegrees());

                telemetry.addData("Pollen PID correction: ", correction);
                pollenTurretDrive.setTargetPosition((int) Math.round(clip(pollenTurretDrive.getCurrentPosition() + (correction * (537.7 / 360)), minTurretRot, maxTurretRot)));

            }
            pollenTurretFlywheel.setVelocity(launcherVelocityTarget);

            if (closestTagDistance < maxDistance) {
                launcherOn = true;
            } else {
                launcherOn = false;

            }

            //Update robot status
            if (currentPathIdx == 0) {
                telemetry.addLine("Robot moving to shoot");
            } else if (currentPathIdx == 1) {
                telemetry.addLine("Robot moving to flower");
            } else {
                telemetry.addLine("Robot moving to shoot");
            }

            //Update turret status
            telemetry.addLine("Launcher vel: " + pollenTurretFlywheel.getVelocity() + "/" + launcherVelocityTarget);
            if (launcherVelocityTarget != 0 && Math.abs(pollenTurretFlywheel.getVelocity() - launcherVelocityTarget) < launcherVelocityTolerance) {
                telemetry.addLine("Launcher reached target velocity");
            } else if (launcherVelocityTarget != 0 && Math.abs(pollenTurretFlywheel.getVelocity() - launcherVelocityTarget) > launcherVelocityTolerance) {
                telemetry.addLine("Launcher spinning up");
            } else if (launcherVelocityTarget == 0 && pollenTurretFlywheel.getVelocity() != 0) {
                telemetry.addLine("Launcher spinning down");
            } else {
                telemetry.addLine("Launcher inactive");
            }

            //Increase current path idx
            boolean pathDone = !waitingForNewPath && follower.completion() >= 0.98;

            if (!paused && pathDone && currentPathIdx < Configuration.autoMode.ordinal()) {
                paused = true;
                timePaused = System.currentTimeMillis();
            }
            if (paused && System.currentTimeMillis() - timePaused > pauseTime) {
                paused = false;
                currentPathIdx++;
                schedule(follow(follower, paths[currentPathIdx]));
                waitingForNewPath = true;
            }
            if (waitingForNewPath && follower.completion() < 0.5) waitingForNewPath = false;

            telemetry.update();
        }

        Configuration.endingPos = localizer.pose();

    }

    public double Interpolate(double min, double max, double in) {
        return min + in * (max - in);
    }

    public static double calculateDistance(double x, double y, double z) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
    }
}
