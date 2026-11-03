package org.firstinspires.ftc.teamcode.comp_opmodes;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.obselete.LauncherProto.lerp;

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
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.RState;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@Autonomous(name = "Auto", group = "Proto", preselectTeleOp = "Fucked teleop")
public class Auto extends LinearOpMode {
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearLeftMotor;
    DcMotor rearRightMotor;
    DcMotor intake;
    DcMotor feedMotor;

    DcMotor pollenTurretFlywheel;
    Servo pollenTurretHood;
    Servo pollenTurretServo1;
    Servo pollenTurretServo2;
    private Follower follower;
    private Localizer localizer;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(56, 90, 90);

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    //DcMotor nectarTurretFlywheel;
    //Servo nectarTurretHood;
    //Servo nectarTurretServo;
    GoBildaPinpointDriver pinpoint;
    boolean intakeOn;
    boolean launcherOn;

    //TODO: tune
    double intakeSpeed = 1;
    double feedSpeed = 1;

    //Hood settings
    double servoHoodDefault = 0.5;
    double hoodMax = 0.5;
    double hoodMin = 0;

    //Launcher settings
    double maxDistance = 1.6;
    double launcherMinSpeed = -0.8;
    double launcherMaxSpeed = -0.1;

    Limelight3A limelight;

    //PID constants
    //TODO: tune
    final double p = 0.01d;
    final double i = 0.0d;
    final double d = 0.001d;

    //PID controller for pollen
    PIDController pollenPID;




    @Override
    public void runOpMode(){
        Scheduler.reset();

        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        localizer.setPose(start);
        follower = Constants.create(hardwareMap, localizer);
        follower.setPose(start);

        long time = System.nanoTime();

        //Initialize pollen PID controller
        pollenPID = new PIDController(p, i, d);

        //Initialize drive motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "fl_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "fr_motor");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "rl_motor");
        rearRightMotor = hardwareMap.get(DcMotor.class, "rr_motor");

        //Initialize intake system
        intake = hardwareMap.get(DcMotor.class, "intake_motor");
        feedMotor = hardwareMap.get(DcMotor.class, "feed_motor");

        //Initialize pollen turret system
        pollenTurretFlywheel = hardwareMap.get(DcMotor.class, "pollen_launcher");
        pollenTurretHood = hardwareMap.get(Servo.class, "pollen_hood");
        pollenTurretServo1 = hardwareMap.get(Servo.class, "pollen_servo_1");
        pollenTurretServo2 = hardwareMap.get(Servo.class, "pollen_servo_2");

        //Nectar not used for first version
        //nectarTurretFlywheel = hardwareMap.get(DcMotor.class, "nectar_launcher");
        //nectarTurretHood = hardwareMap.get(Servo.class, "nectar_hood");
        //nectarTurretServo = hardwareMap.get(Servo.class, "nectar_servo");

        //Initialize pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,start.x(), start.y(),AngleUnit.DEGREES, start.heading()));

        //Initialize limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        //Swap limelight to pipeline 0 (Blue I believe)
        limelight.pipelineSwitch(0);

        waitForStart();

        //Predefine pollen hood position
        pollenTurretHood.setPosition(0.5);

        schedule(follow(follower, path1()));

        //Main event loop
        while(opModeIsActive()){
            follower.update();
            Scheduler.execute();

            time = System.nanoTime();

            if(time > 30000){
                RState.position = follower.pose();
                requestOpModeStop();
            }

            //Update pinpoint odometry
            pinpoint.update();

            //Fetch limelight
            LLResult result = limelight.getLatestResult();

            //Check for limelight disconnect and warn user
            if(!limelight.isConnected()){
                telemetry.addLine("Limelight disconnected. Robot reboot recommended. Check wiring.");
            }

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            //Init variables for closest tag
            LLResultTypes.FiducialResult closestTag = null;
            double closestTagDistance = Double.POSITIVE_INFINITY;

            //Find closest april tag
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                double distance = calculateDistance(fiducial.getRobotPoseTargetSpace().getPosition().x, fiducial.getRobotPoseTargetSpace().getPosition().y, fiducial.getRobotPoseTargetSpace().getPosition().z);
                if(distance < closestTagDistance){
                    closestTagDistance = distance;
                    closestTag = fiducial;
                }
            }

            //Check for any tags
            if(closestTag == null){
                telemetry.addLine("No tags detected");

                //Return servo hood to midpoint
                pollenTurretServo1.setPosition(servoHoodDefault);
                pollenTurretServo2.setPosition(servoHoodDefault);
            }
            else{

                telemetry.addData("Targeting tag: ", closestTag.getFiducialId());

                //Rough estimates for flywheel power & hood position
                pollenTurretHood.setPosition(lerp(hoodMin, hoodMax, maxDistance/closestTagDistance));

                //Toggle launcher hardware
                if(launcherOn){
                    pollenTurretFlywheel.setPower(lerp(launcherMinSpeed, launcherMaxSpeed, maxDistance/closestTagDistance));
                }
                else{
                    pollenTurretFlywheel.setPower(0);
                }

                //PID input for targeting
                pollenPID.setSetPoint(0.0);
                double correction = pollenPID.calculate(closestTag.getTargetXPixels());

                telemetry.addData("Pollen PID correction: ", correction);

                pollenTurretServo1.setPosition(correction);
                pollenTurretServo2.setPosition(correction);
            }

            if(closestTagDistance < 1){
                launcherOn = true;
            }
            else{
                launcherOn = false;
            }

            telemetry.addData("Pinpoint pos: ", pinpoint.getPosition());
            //Enable/Disable intake system
            if(intakeOn){
                intake.setPower(intakeSpeed);
                feedMotor.setPower(feedSpeed);
            }
            else{
                intake.setPower(0);
                feedMotor.setPower(0);
            }
            telemetry.update();
        }
    }

    public Pose Pose2Dto3D(Pose2D pose){
        return new Pose(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.DEGREES));
    }

    public double Interpolate(double min, double max, double in){
        return min + in * (max - in);
    }

    public static double calculateDistance(double x, double y, double z) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
    }
}
