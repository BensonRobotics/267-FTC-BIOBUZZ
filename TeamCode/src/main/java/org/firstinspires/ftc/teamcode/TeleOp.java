package org.firstinspires.ftc.teamcode;

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

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearLeftMotor;
    DcMotor rearRightMotor;
    DcMotor intake;

    DcMotor pollenTurretFlywheel;
    Servo pollenTurretHood;
    Servo pollenTurretServo;

    DcMotor nectarTurretFlywheel;
    Servo nectarTurretHood;
    Servo nectarTurretServo;
    GoBildaPinpointDriver pinpoint;
    boolean intakeOn;
    boolean launcherOn;
    double intakeSpeed = 1;
    double driveSpeed = 1;
    Limelight3A limelight;

    @Override
    public void runOpMode(){

        //Initialize motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "fl_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "fr_motor");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "rl_motor");
        rearRightMotor = hardwareMap.get(DcMotor.class, "rr_motor");

        intake = hardwareMap.get(DcMotor.class, "intake_motor");

        pollenTurretFlywheel = hardwareMap.get(DcMotor.class, "pollen_launcher");
        pollenTurretHood = hardwareMap.get(Servo.class, "pollen_hood");
        pollenTurretServo = hardwareMap.get(Servo.class, "pollen_servo");

        nectarTurretFlywheel = hardwareMap.get(DcMotor.class, "nectar_launcher");
        nectarTurretHood = hardwareMap.get(Servo.class, "nectar_hood");
        nectarTurretServo = hardwareMap.get(Servo.class, "nectar_servo");

        //Initialize pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        //Initialize limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        //Swap limelight to pipeline 0 (Blue I believe)
        limelight.pipelineSwitch(0);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        //Predefine pollen hood position
        pollenTurretHood.setPosition(0.5);


        while(opModeIsActive()){
            pinpoint.update();

            LLResult result = limelight.getLatestResult();

            if(!limelight.isConnected()){
                telemetry.addLine("Limelight disconnected. Robot reboot recommended. Check wiring.");
            }
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            LLResultTypes.FiducialResult closestTag = null;
            double closestTagDistance = Double.POSITIVE_INFINITY;

            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                double distance = calculateDistance(fiducial.getRobotPoseTargetSpace().getPosition().x, fiducial.getRobotPoseTargetSpace().getPosition().y, fiducial.getRobotPoseTargetSpace().getPosition().z);
                if(distance < closestTagDistance){
                    closestTagDistance = distance;
                    closestTag = fiducial;
                }
            }
            if(closestTag == null){
                telemetry.addLine("No tags detected");
                pollenTurretServo.setPosition(0.5);
            }
            else{
                telemetry.addData("Targeting tag: ", closestTag.getFiducialId());

                if(closestTag.getTargetXDegrees() < -1){
                    pollenTurretServo.setPosition(pollenTurretServo.getPosition() + 0.1d);
                    nectarTurretServo.setPosition(nectarTurretServo.getPosition() + 0.1d);
                }
                else if(closestTag.getTargetXDegrees() > 1){
                    pollenTurretServo.setPosition(pollenTurretServo.getPosition() - 0.1d);
                    nectarTurretServo.setPosition(nectarTurretServo.getPosition() - 0.1d);
                }
            }

            telemetry.addData("Pinpoint pos: ", pinpoint.getPosition());

            if(gamepad1.aWasPressed()){
                intakeOn = !intakeOn;
            }
            if(gamepad1.bWasPressed()){
                launcherOn = !launcherOn;
            }

            if(intakeOn){
                intake.setPower(intakeSpeed);
            }
            else{
                intake.setPower(0);
            }
            if(launcherOn){
                pollenTurretFlywheel.setPower(-1);
                nectarTurretFlywheel.setPower(-1);
            }
            else{
                pollenTurretFlywheel.setPower(0);
                nectarTurretFlywheel.setPower(0);
            }

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
