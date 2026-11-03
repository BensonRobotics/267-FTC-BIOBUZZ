package org.firstinspires.ftc.teamcode.comp_opmodes;

import static org.firstinspires.ftc.teamcode.obselete.LauncherProto.lerp;

import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.RState;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDController;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Proto")
public class TeleOp extends LinearOpMode {
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearLeftMotor;
    DcMotor rearRightMotor;
    DcMotor intake;
    DcMotor feedMotor;

    DcMotorEx pollenTurretFlywheel;
    DcMotorEx pollenTurretDrive;
    Servo pollenTurretHood;
    //Servo pollenTurretServo2;
    Localizer localizer;

    //DcMotor nectarTurretFlywheel;
    //Servo nectarTurretHood;
    //Servo nectarTurretServo;
    GoBildaPinpointDriver pinpoint;
    boolean intakeOn;
    boolean launcherOn;

    //TODO: tune
    double intakeSpeed = 1;
    double driveSpeed = 1;
    double feedSpeed = 1;

    //Hood settings
    double servoHoodDefault = 0.5;
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

    public Pose2D startingPosition;

    @Override
    public void runOpMode(){
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        localizer.setPose(RState.position);

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
        pollenTurretFlywheel = hardwareMap.get(DcMotorEx.class, "pollen_launcher");
        pollenTurretHood = hardwareMap.get(Servo.class, "pollen_hood");
        pollenTurretDrive = hardwareMap.get(DcMotorEx.class, "pollen_turret_drive");

        //Reset pollen turret drive motor encoder
        pollenTurretDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pollenTurretDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Nectar not used for lm1
        //nectarTurretFlywheel = hardwareMap.get(DcMotor.class, "nectar_launcher");
        //nectarTurretHood = hardwareMap.get(Servo.class, "nectar_hood");
        //nectarTurretServo = hardwareMap.get(Servo.class, "nectar_servo");

        //Initialize pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setPosition(startingPosition);

        //Initialize limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        //Swap limelight to pipeline 0 (Blue I believe)
        limelight.pipelineSwitch(0);

        waitForStart();

        //Predefine pollen hood position
        pollenTurretHood.setPosition(0.5);

        //Main event loop
        while(opModeIsActive()){

            //Update pinpoint odometry
            pinpoint.update();

            localizer.update();

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
                pollenTurretFlywheel.setPower(0);
                //pollenTurretServo2.setPosition(servoHoodDefault);
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
                        feedMotor.setPower(intakeSpeed);
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

            telemetry.addData("Pinpoint pos: ", localizer.pose());

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

            //Some really cool movement code I stole from bartholomew
            //double heading = pinpoint.getHeading(AngleUnit.RADIANS);
            double heading = localizer.pose().heading();

            double fr = 0;
            double fl = 0;
            double br = 0;
            double bl = 0;

            double leftStickY = gamepad1.left_stick_y;
            double leftStickX = gamepad1.left_stick_x;
            double rightStickX = -gamepad1.right_stick_x;
            double rightStickY = gamepad1.right_stick_y;

            double theta = Math.atan2(-leftStickX, leftStickY);
            theta += heading;

            telemetry.addData("IMU Yaw ", heading);

            double magnitude = Math.hypot(-leftStickX, leftStickY);

            double leftPower = leftStickY - leftStickX;
            double rightPower = -leftStickY - leftStickX;

            double rX = -gamepad1.right_stick_x;
            double flbr = magnitude * Math.sin(theta + (0.25 * Math.PI));
            double frbl = magnitude * Math.sin(theta - (0.25 * Math.PI));

            fr = frbl - rX;
            fl = flbr + rX;
            br = flbr - rX;
            bl = frbl + rX;

            double denominator = Math.max(Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(br), Math.abs(bl))), 1);

            fr /= -denominator;
            fl /= -denominator;
            br /= denominator;
            bl /= denominator;

            frontRightMotor.setPower(fr);
            frontLeftMotor.setPower(fl);
            rearRightMotor.setPower(br);
            rearLeftMotor.setPower(bl);
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
