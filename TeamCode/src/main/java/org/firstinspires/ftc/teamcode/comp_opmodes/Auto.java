package org.firstinspires.ftc.teamcode.comp_opmodes;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
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
    double feedSpeed = 1;

    //Hood settings
    //TODO: tune
    double servoHoodDefault = 0.5;
    double hoodMax = 0.5;
    double hoodMin = 0;

    //Launcher settings
    double maxDistance = 1.6;

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

    private Vector2D center = new Vector2D(70, 70);
    private TurretState pollenDriveState = TurretState.UNDEFINED;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    //Paths

        //Blue Starting Left of Park (blp)
        private final Pose blpstart = poseFactory.of(132.8953, 8.5941, 180);
        private final Pose blppath1 = poseFactory.of(88.1387, 111.4928, -90);
        private final Pose blppath1Control1 = poseFactory.of(122.278, 123.9244, 0);
        private final Pose blppath2Start = poseFactory.of(88.1387, 111.4928, -90);
        private final Pose blppath2 = poseFactory.of(94.6568, 11.683, 90);
        private final Pose blppath2Control1 = poseFactory.of(47.5686, 70.4204, 0);
        private final Pose blppath3 = poseFactory.of(84.8848, 35.2844, 90);

        public Path blppath1() {
            return Paths.curve(blpstart, blppath1Control1, blppath1).linear(blpstart, blppath1);
        }

        public Path blppath2() {
            return Paths.curve(blppath2Start, blppath2Control1, blppath2).linear(blppath2Start, blppath2);
        }

        public Path blppath3() {
            return Paths.line(blppath2, blppath3).constant(blppath3);
        }

        public Path[] blpPaths = new Path[] {blppath1(), blppath2(), blppath3()};
        //Blue Starting Right Of Park (brp)
        private final Pose brpstart = poseFactory.of(133.2537, 57.1546, 180);
        private final Pose brppath1 = poseFactory.of(85.3259, 114.5664, -90);
        private final Pose brppath1Control1 = poseFactory.of(117.201, 111.5048, 0);
        private final Pose brppath2Start = poseFactory.of(85.3259, 114.5664, -90);
        private final Pose brppath2 = poseFactory.of(131.0831, 94.5287, 180);
        private final Pose brppath3Start = poseFactory.of(131.0831, 94.5287, 180);
        private final Pose brppath3 = poseFactory.of(81.8934, 20.2173, 90);
        private final Pose brppath3Control1 = poseFactory.of(94.8611, 138.3269, 0);
        private final Pose brppath3Control2 = poseFactory.of(40.9743, 93.6964, 0);

        public Path brppath1() {
            return Paths.curve(brpstart, brppath1Control1, brppath1).linear(brpstart, brppath1);
        }

        public Path brppath2() {
            return Paths.line(brppath2Start, brppath2).linear(brppath2Start, brppath2);
        }

        public Path brppath3() {
            return Paths.curve(brppath3Start, brppath3Control1, brppath3Control2, brppath3).linear(brppath3Start, brppath3);
        }
        public Path[] brpPaths = new Path[] {brppath1(), brppath2(), brppath3()};


        //Red Starting Left Of Park (rlp)
        private final Pose rlpstart = poseFactory.of(8.6172, 130.0891, 0);
        private final Pose rlppath1 = poseFactory.of(59.3351, 28.4012, 90);
        private final Pose rlppath1Control1 = poseFactory.of(106.0495, 118.0274, 0);
        private final Pose rlppath2Start = poseFactory.of(59.3351, 28.4012, 90);
        private final Pose rlppath2 = poseFactory.of(46.5033, 130.9306, -90);
        private final Pose rlppath2Control1 = poseFactory.of(86.8748, 82.4669, 0);
        private final Pose rlppath3 = poseFactory.of(57.8029, 111.5747, -90);

        public Path rlppath1() {
            return Paths.curve(rlpstart, rlppath1Control1, rlppath1).linear(rlpstart, rlppath1);
        }

        public Path rlppath2() {
            return Paths.curve(rlppath2Start, rlppath2Control1, rlppath2).linear(rlppath2Start, rlppath2);
        }

        public Path rlppath3() {
            return Paths.line(rlppath2, rlppath3).constant(rlppath3);
        }
        public Path[] rlpPaths = new Path[] {rlppath1(), rlppath2(), rlppath2()};


        //Red Starting Right Of Park (rrp)
        private final Pose rrpstart = poseFactory.of(8.2214, 13.6113, 0);
        private final Pose rrppath1 = poseFactory.of(56, 35.4344, 90);
        private final Pose rrppath2Start = poseFactory.of(56, 35.4344, 90);
        private final Pose rrppath2 = poseFactory.of(10.2355, 47.1108, 0);
        private final Pose rrppath3Start = poseFactory.of(10.2355, 47.1108, 0);
        private final Pose rrppath3 = poseFactory.of(57.1564, 113.5619, -90);
        private final Pose rrppath3Control1 = poseFactory.of(64.1036, 5.5443, 0);
        private final Pose rrppath3Control2 = poseFactory.of(98.2945, 45.3265, 0);

        public Path rrppath1() {
            return Paths.line(rrpstart, rrppath1).linear(rrpstart, rrppath1);
        }

        public Path rrppath2() {
            return Paths.line(rrppath2Start, rrppath2).linear(rrppath2Start, rrppath2);
        }

        public Path rrppath3() {
            return Paths.curve(rrppath3Start, rrppath3Control1, rrppath3Control2, rrppath3).linear(rrppath3Start, rrppath3);
        }

        public Path[] rrpPaths = new Path[] {rrppath1(), rrppath2(), rrppath3()};



    @Override
    public void runOpMode(){
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
        while(opModeInInit()){
            //Move menu selector
            if(gamepad1.dpadDownWasPressed()){
                menuHandler.ChangeSelection(-1);
            }
            else if(gamepad1.dpadUpWasPressed()){
                menuHandler.ChangeSelection(1);
            }
            else if(gamepad1.dpadRightWasPressed()){
                menuHandler.EditCurrentSelection();
            }

            //Finish menu and export
            if(gamepad1.startWasPressed()){
                configurationCompleted = true;
                MenuOptionExport[] menuExports = menuHandler.ExportSelections();

                Configuration.alliance = Alliance.valueOf(menuExports[0].selection);
                if(Objects.equals(menuExports[1].selection, "Left Of Park")){
                    Configuration.startingPos = StartingPos.LeftOfPark;
                }
                else{
                    Configuration.startingPos = StartingPos.RightOfPark;
                }

                if(Objects.equals(menuExports[2].selection, "Just Shoot")){
                    Configuration.autoMode = AutoMode.JustShoot;
                }
                else if(Objects.equals(menuExports[2].selection, "Shoot Then Intake")){
                    Configuration.autoMode = AutoMode.ShootThenEat;
                }
                else{
                    Configuration.autoMode = AutoMode.ShootEatShoot;
                }
                menuHandler.LockOptions();
            }
            menuHandler.Redraw();
            telemetry.update();
        }
        if(!configurationCompleted){
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

        if(Configuration.alliance == Alliance.Blue && Configuration.startingPos == StartingPos.LeftOfPark){
            start = blpstart;
            paths = blpPaths;
        }
        else if(Configuration.alliance == Alliance.Blue && Configuration.startingPos == StartingPos.RightOfPark){
            start = brpstart;
            paths = brpPaths;
        }
        else if(Configuration.alliance == Alliance.Red && Configuration.startingPos == StartingPos.RightOfPark){
            start = rrpstart;
            paths = rrpPaths;
        }
        else /*if(Configuration.alliance == Alliance.Red && Configuration.startingPos == StartingPos.LeftOfPark) */{
            start = rlpstart;
            paths = rlpPaths;
        }
        Localizer localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        localizer.setPose(start);
        Follower follower = Constants.create(hardwareMap, localizer);
        follower.setPose(start);


        if(Configuration.alliance == Alliance.Blue){
            limelight.pipelineSwitch(0);
        }
        else if(Configuration.alliance == Alliance.Red){
            limelight.pipelineSwitch(1);
        }

        waitForStart();

        pollenTurretDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pollenTurretFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Predefine pollen hood position
        pollenTurretHood.setPosition(0.5);

        schedule(follow(follower, paths[0]));

        //Main event loop
        while(opModeIsActive()){

            follower.update();
            Scheduler.execute();

            time = System.nanoTime();

            if(time > 30000){
                Configuration.endingPos = localizer.pose();
                requestOpModeStop();
            }
            else if(isStopRequested()){
                Configuration.endingPos = localizer.pose();
            }

            //Fetch limelight
            LLResult result = limelight.getLatestResult();

            //Check for limelight disconnect and warn user
            if(!limelight.isConnected()){
                telemetry.addLine("WARN: Limelight disconnected. Robot reboot recommended. Check wiring.");
            }
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            //Init variables for closest tag
            LLResultTypes.FiducialResult closestTag = null;
            double closestTagDistance = Double.POSITIVE_INFINITY;

            //Find closest april tag
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                Pose3D robotPose = fiducial.getRobotPoseTargetSpace();

                double x = robotPose.getPosition().x;
                double y = robotPose.getPosition().y;

                double distance = Math.hypot(x, y);
                if(distance < closestTagDistance){
                    closestTagDistance = distance;
                    closestTag = fiducial;
                }
            }

            if(Math.abs(pollenTurretDrive.getCurrentPosition()) > 537.7){
                pollenDriveState = TurretState.RETURNING_TO_ZERO;
            }

            if(pollenDriveState == TurretState.RETURNING_TO_ZERO){
                pollenTurretDrive.setTargetPosition(0);
                if(Math.abs(pollenTurretDrive.getCurrentPosition()) < 1){
                    pollenDriveState = TurretState.UNDEFINED;
                }
            }

            //Check for any tags
            if(closestTag == null){
                launcherVelocityTarget = 0;
                if(pollenDriveState != TurretState.RETURNING_TO_ZERO){
                    pollenDriveState = TurretState.RUNNING_TO_POSITION_NOT_LOCKED;
                    Vector2D directionalVector = new Vector2D(localizer.pose().x() - center.getX(), localizer.pose().y() - center.getY());
                    double angle = Math.atan2(directionalVector.getX(), directionalVector.getY());

                    pollenTurretDrive.setTargetPosition((int) Math.round((angle / (2 * Math.PI)) * 537.7));
                }

            }
            else{
                if(pollenDriveState != TurretState.RETURNING_TO_ZERO) {
                    pollenDriveState = TurretState.RUNNING_TO_POSITION_LOCK;
                    telemetry.addData("Targeting tag: ", closestTag.getFiducialId());

                    //Rough estimates for flywheel power & hood position
                    pollenTurretHood.setPosition(lerp(hoodMin, hoodMax, maxDistance / closestTagDistance));

                    //Toggle launcher hardware
                    if (launcherOn) {
                        launcherVelocityTarget = lerp(launcherMinSpeed, launcherMaxSpeed, maxDistance / closestTagDistance);

                        //Only enable feed when pollen flywheel is up to speed
                        if (Math.abs(pollenTurretFlywheel.getVelocity() - launcherVelocityTarget) < launcherVelocityTolerance) {
                            feedMotor.setPower(intakeSpeed);
                        }
                    } else {
                        launcherVelocityTarget = 0;
                    }

                    //PID input for targeting
                    pollenPID.setSetPoint(0.0);
                    double correction = pollenPID.calculate(closestTag.getTargetXPixels());

                    telemetry.addData("Pollen PID correction: ", correction);

                    pollenTurretDrive.setTargetPosition((int) Math.round(pollenTurretDrive.getCurrentPosition() + correction));
                }
            }
            pollenTurretFlywheel.setVelocity(launcherVelocityTarget);

            if(closestTagDistance < 1){
                launcherOn = true;
            }
            else{
                launcherOn = false;
            }

            //Enable/Disable intake system
            if(intakeOn){
                intake.setPower(intakeSpeed);
                feedMotor.setPower(feedSpeed);
            }
            else{
                intake.setPower(0);
                feedMotor.setPower(0);
            }

            //Update robot status
            if(currentPathIdx == 0){
                telemetry.addLine("Robot moving to shoot");
            }
            else if(currentPathIdx == 1){
                telemetry.addLine("Robot moving to flower");
            }
            else{
                telemetry.addLine("Robot moving to shoot");
            }

            //Update turret status
            telemetry.addLine("Launcher vel: " + pollenTurretDrive.getVelocity() + "/" + launcherVelocityTarget);
            if(launcherVelocityTarget != 0 && Math.abs(pollenTurretFlywheel.getVelocity() - launcherVelocityTarget) < launcherVelocityTolerance){
                telemetry.addLine("Launcher reached target velocity");
            }
            else if(launcherVelocityTarget != 0 && Math.abs(pollenTurretFlywheel.getVelocity() - launcherVelocityTarget) > launcherVelocityTolerance){
                telemetry.addLine("Launcher spinning up");
            }
            else if(launcherVelocityTarget == 0 && pollenTurretFlywheel.getVelocity() != 0){
                telemetry.addLine("Launcher spinning down");
            }
            else{
                telemetry.addLine("Launcher inactive");
            }

            //Increase current path idx
            if(follower.completion() == 1 && currentPathIdx < Configuration.autoMode.ordinal() && (Math.abs(System.currentTimeMillis() - timePaused) > pauseTime)){
                timePaused = System.currentTimeMillis();
                currentPathIdx++;
                schedule(follow(follower, paths[currentPathIdx]));
            }

            telemetry.update();
        }

    }
    public double Interpolate(double min, double max, double in){
        return min + in * (max - in);
    }

    public static double calculateDistance(double x, double y, double z) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
    }
}
