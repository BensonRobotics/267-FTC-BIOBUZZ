package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.LauncherProto.lerp;
import static java.sql.Types.NULL;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDController;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearLeftMotor;
    DcMotor rearRightMotor;
    DcMotor intake;
    DcMotor feedMotor;

    DcMotor pollenTurretFlywheel;
    Servo pollenTurretHood;
    Servo pollenTurretServo;

    //DcMotor nectarTurretFlywheel;
    //Servo nectarTurretHood;
    //Servo nectarTurretServo;
    GoBildaPinpointDriver pinpoint;
    boolean intakeOn;
    boolean launcherOn;
    double intakeSpeed = 1;
    double driveSpeed = 1;
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
        pollenTurretServo = hardwareMap.get(Servo.class, "pollen_servo");

        //Nectar not used for first version
        //nectarTurretFlywheel = hardwareMap.get(DcMotor.class, "nectar_launcher");
        //nectarTurretHood = hardwareMap.get(Servo.class, "nectar_hood");
        //nectarTurretServo = hardwareMap.get(Servo.class, "nectar_servo");

        //Initialize pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

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
                pollenTurretServo.setPosition(0.5);
            }
            else{

                telemetry.addData("Targeting tag: ", closestTag.getFiducialId());

                //Rough estimates for flywheel power & hood position
                pollenTurretHood.setPosition(lerp(0, 0.5, 1.6/closestTagDistance));
                pollenTurretFlywheel.setPower(lerp(-0.8d, -0.1d, 1.6/closestTagDistance));

                //PID input for targeting
                pollenPID.setSetPoint(0.0);
                double correction = pollenPID.calculate(closestTag.getTargetXPixels());

                telemetry.addData("Pollen PID correction: ", correction);

                pollenTurretServo.setPosition(correction);
            }

            telemetry.addData("Pinpoint pos: ", pinpoint.getPosition());

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
                feedMotor.setPower(1);
            }
            else{
                intake.setPower(0);
                feedMotor.setPower(0);
            }

            //Enable/Disable launcher system
            if(launcherOn){
                pollenTurretFlywheel.setPower(-1);
                //nectarTurretFlywheel.setPower(-1);
            }
            else{
                pollenTurretFlywheel.setPower(0);
                //nectarTurretFlywheel.setPower(0);
            }

            //Some really cool movement code I stole from bartholomew
            double heading = pinpoint.getHeading(AngleUnit.RADIANS);

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

            frontRightMotor.setPower(Interpolate(0, driveSpeed, fr));
            frontLeftMotor.setPower(Interpolate(0, driveSpeed, fl));
            rearRightMotor.setPower(Interpolate(0, driveSpeed, br));
            rearLeftMotor.setPower(Interpolate(0, driveSpeed, bl));
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
