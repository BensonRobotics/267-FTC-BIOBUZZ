package org.firstinspires.ftc.teamcode.comp_opmodes;

import static org.firstinspires.ftc.teamcode.obselete.LauncherProto.lerp;

import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Configuration;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.seattlesolvers.solverslib.controller.PIDController;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Comp")
public class TeleOp extends LinearOpMode {
    MotorEx frontLeftMotor;
    MotorEx frontRightMotor;
    MotorEx rearLeftMotor;
    MotorEx rearRightMotor;
    DcMotor intake;
    DcMotor feedMotor;

    DcMotorEx pollenTurretFlywheel;
    DcMotorEx pollenTurretDrive;
    Servo pollenTurretHood;
    Localizer localizer;
    boolean intakeOn;
    boolean launcherOn;

    //TODO: tune
    double intakeSpeed = 1;
    double feedSpeed = 1;

    //Hood settings
    double hoodMax = 0.5;
    double hoodMin = 0;

    //Launcher settings
    double maxDistance = 1.6;

    //Speeds are in TPM (Ticks Per Minute) counts
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
    public IMU imu;
    public MotorEx[] motors;

    public Pose2D startingPosition;
    public Command driverControlled;

    public int fieldCenterX = 72;
    public int fieldCenterY = 72;


    @Override
    public void runOpMode(){
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        if(Configuration.endingPos != null){
            localizer.setPose(Configuration.endingPos);
            Configuration.endingPos = null;
        }

        //Initialize pollen PID controller
        pollenPID = new PIDController(p, i, d);

        //Initialize drive motors
        frontLeftMotor = new MotorEx("fl_motor");
        frontRightMotor = new MotorEx("fr_motor");
        rearLeftMotor = new MotorEx("rl_motor");
        rearRightMotor = new MotorEx("rr_motor");

        //Initialize intake system
        intake = hardwareMap.get(DcMotor.class, "intake_motor");
        feedMotor = hardwareMap.get(DcMotor.class, "feed_motor");

        //Initialize pollen turret system
        pollenTurretFlywheel = hardwareMap.get(DcMotorEx.class, "pollen_launcher");
        pollenTurretHood = hardwareMap.get(Servo.class, "pollen_hood");
        pollenTurretDrive = hardwareMap.get(DcMotorEx.class, "pollen_turret_drive");

        //Reset pollen turret drive motor encoder
        pollenTurretDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pollenTurretDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        GamepadEx gamepad = new GamepadEx(gamepad1, 0, 0, 0, true);

        motors = new MotorEx[] { frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor };
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        //Initialize limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();


        if(Configuration.alliance == Alliance.Blue){
            limelight.pipelineSwitch(0);
        }
        else if(Configuration.alliance == Alliance.Red){
            limelight.pipelineSwitch(1);
        }

        waitForStart();

        //Predefine pollen hood position
        pollenTurretHood.setPosition(0.5);

        //Invoke holonomic drivetrain
        driverControlled = new MecanumDriverControlled(motors, gamepad, false, imu);
        driverControlled.invoke();

        //Main event loop
        while(opModeIsActive()){
            localizer.update();

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

            //TODO: Implement system for pointing turret towards the middle of the field from Auto.java
            //Check for any tags
            if(closestTag == null){
                pollenTurretFlywheel.setPower(0);
            }
            else{

                telemetry.addData("Targeting tag: ", closestTag.getFiducialId());

                //Rough estimates for flywheel power & hood position
                pollenTurretHood.setPosition(lerp(hoodMin, hoodMax, maxDistance/closestTagDistance));

                //Toggle launcher hardware
                if(launcherOn){
                    launcherVelocityTarget = lerp(launcherMinSpeed, launcherMaxSpeed, maxDistance/closestTagDistance);

                    //Only enable feed when pollen flywheel is up to speed
                    if(Math.abs(pollenTurretFlywheel.getVelocity() - launcherVelocityTarget) < launcherVelocityTolerance){
                        feedMotor.setPower(feedSpeed);
                    }
                }
                else{
                    pollenTurretFlywheel.setVelocity(0);
                }

                //PID input for targeting
                pollenPID.setSetPoint(0.0);
                double correction = pollenPID.calculate(closestTag.getTargetXPixels());

                telemetry.addData("Pollen PID correction: ", correction);

                pollenTurretDrive.setTargetPosition((int)Math.round(pollenTurretDrive.getCurrentPosition() + correction));
            }
            pollenTurretFlywheel.setVelocity(launcherVelocityTarget);
            //Toggle intake
            if(gamepad1.aWasPressed()){
                intakeOn = !intakeOn;
            }
            if(gamepad1.bWasPressed()){
                launcherOn = !launcherOn;
            }

            //Enable/Disable intake system
            if(intakeOn){
                intake.setPower(intakeSpeed);
                feedMotor.setPower(feedSpeed);
            }
            else{
                intake.setPower(0);
                if(!launcherOn){
                    feedMotor.setPower(0);
                }
            }
            telemetry.addData("Position: ", localizer.pose());
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
