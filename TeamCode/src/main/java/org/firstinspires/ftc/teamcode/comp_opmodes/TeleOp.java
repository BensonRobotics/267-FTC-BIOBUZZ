package org.firstinspires.ftc.teamcode.comp_opmodes;

import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.obselete.LauncherProto.lerp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoMode;
import org.firstinspires.ftc.teamcode.util.Configuration;
import org.firstinspires.ftc.teamcode.util.MenuHandler;
import org.firstinspires.ftc.teamcode.util.MenuOption;
import org.firstinspires.ftc.teamcode.util.StartingPos;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDController;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Comp")

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
    Localizer localizer;
    boolean intakeForwards;
    boolean launcherOn;
    boolean intakeBackwards;

    //TODO: tune
    final double intakeSpeed = -0.75;
    final double feedSpeed = -0.8;

    //Hood settings
    double hoodMax = 0.7;
    double hoodMin = 0.2;

    //Launcher settings
    final double maxDistance = 1.1;

    //Speeds are in TPS (Ticks Per Seconds) counts
    double launcherMinSpeed = -2200;
    double launcherMaxSpeed = -1700;

    final double maxTurretRot = 750;
    final double minTurretRot = -975;

    final double launcherVelocityTolerance = 100;
    double launcherVelocityTarget = 0;

    Limelight3A limelight;

    //PID constants
    //TODO: tune

    public static volatile double P = 2d;
    public static volatile double I = 0.0d;
    public static volatile double D = 0.0d;

    //PID controller for pollen
    PIDController pollenPID;
    public IMU imu;
    boolean feedOverride = false;
    boolean turretManualOverride = false;
    double manualOverrideSpeed = 0;
    boolean targetCenter = false;


    @Override
    public void runOpMode(){
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        pollenTurretDrive = hardwareMap.get(DcMotorEx.class, "pollen_turret_drive");

        if(Configuration.endingPos != null && Configuration.autoMode != AutoMode.Disabled){
            telemetry.addLine("Received config from autonomous.");
            telemetry.update();
            localizer.setPose(Configuration.endingPos);
            Configuration.endingPos = null;
        }
        else{
            telemetry.addLine("Align turret with markings");
            telemetry.update();
            pollenTurretDrive.setTargetPosition(0);
            //Reset pollen turret drive motor encoder
            pollenTurretDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pollenTurretDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pollenTurretDrive.setPower(1);


            if(Configuration.alliance == Alliance.Red){
                if(Configuration.startingPos == StartingPos.LeftOfPark){
                    localizer.setPose(Auto.rlpstart);
                }
                else if(Configuration.startingPos == StartingPos.RightOfPark){
                    localizer.setPose(Auto.rrpstart);
                }
            }
            else if(Configuration.alliance == Alliance.Blue){
                if(Configuration.startingPos == StartingPos.LeftOfPark){
                    localizer.setPose(Auto.blpstart);
                }
                else if(Configuration.startingPos == StartingPos.RightOfPark){
                    localizer.setPose(Auto.brpstart);
                }
            }

        }

        //Initialize pollen PID controller
        pollenPID = new PIDController(P, I, D);

        //Initialize drive motors
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fl_motor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fr_motor");
        rearLeftMotor = hardwareMap.get(DcMotorEx.class, "rl_motor");
        rearRightMotor = hardwareMap.get(DcMotorEx.class, "rr_motor");

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

        //Initialize intake system
        intake = hardwareMap.get(DcMotor.class, "intake_motor");
        feedMotor = hardwareMap.get(DcMotor.class, "feed_motor");

        //Initialize pollen turret system
        pollenTurretFlywheel = hardwareMap.get(DcMotorEx.class, "pollen_launcher");
        pollenTurretHood = hardwareMap.get(Servo.class, "pollen_hood");


        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
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

        //Main event loop
        while(opModeIsActive()){
            //dashboard.updateConfig();
            pollenPID = new PIDController(P, I, D);


            //pollenPID.reset();
            telemetry.addData("P: ", P);
            telemetry.addData("I: ", I);
            telemetry.addData("D: ", D);
            localizer.update();
            double y  = -gamepad1.left_stick_y;
            double x  = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.optionsWasPressed()) imu.resetYaw();
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            frontLeftMotor.setPower((rotY + rotX + rx) / denom);
            rearLeftMotor.setPower((rotY - rotX + rx) / denom);
            frontRightMotor.setPower((rotY - rotX - rx) / denom);
            rearRightMotor.setPower((rotY + rotX - rx) / denom);
            //Fetch limelight
            LLResult result = limelight.getLatestResult();

            //Check for limelight disconnect and warn user
            if(!limelight.isConnected()){
                telemetry.addLine("WARN: Limelight disconnected. Robot reboot recommended. Check wiring.");
            }

            if(turretManualOverride){
                telemetry.addLine("WARN: Turret manual override activated. Restart program if this was unintentional.");
            }

            if(!turretManualOverride) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                telemetry.addData("Detected fiducials: ", (long) fiducials.size());

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

                //TODO: Implement system for pointing turret towards the middle of the field from Auto.java
                //Check for any tags

                if (closestTag == null) {
                    if(targetCenter){
                        double bearing = Math.toDegrees(Math.atan2(70 - localizer.pose().x(), 70 - localizer.pose().y()));
                        double rel = AngleUnit.normalizeDegrees(bearing - Math.toDegrees(localizer.pose().heading()));
                        pollenTurretDrive.setTargetPosition((int) Math.round(Range.clip(rel * (537.7 / 360), minTurretRot, maxTurretRot)));
                    }
                    launcherVelocityTarget = 0;
                    pollenPID.reset();
                } else {
                    telemetry.addData("Targeting tag: ", closestTag.getFiducialId());

                    //Rough estimates for flywheel power & hood position
                    pollenTurretHood.setPosition(lerp(hoodMax, hoodMin, closestTagDistance / maxDistance));
                    //Toggle launcher hardware
                    if (launcherOn) {
                        launcherVelocityTarget = lerp(launcherMaxSpeed, launcherMinSpeed, closestTagDistance / maxDistance);
                        //launcherVelocityTarget = 0.8;

                        //Only enable feed when pollen flywheel is up to speed
                        //if (Math.abs(pollenTurretFlywheel.getVelocity() - launcherVelocityTarget) < launcherVelocityTolerance) {
                        //    feedMotor.setPower(feedSpeed);
                        //}
                    } else {
                        launcherVelocityTarget = 0;
                    }

                    //PID input for targeting
                    pollenPID.setSetPoint(0.0);
                    double correction = pollenPID.calculate(closestTag.getTargetXDegrees());
                    telemetry.addData("Correction: ", correction);
                    pollenTurretDrive.setTargetPosition((int) Math.round(Range.clip(pollenTurretDrive.getCurrentPosition() + correction, minTurretRot, maxTurretRot)));
                }
                if(gamepad1.b){
                    launcherOn = true;
                }
                else{
                    launcherOn = false;
                }
                pollenTurretFlywheel.setVelocity(launcherVelocityTarget);
            }

            if(turretManualOverride){
                telemetry.addData("Manual override speed: ", manualOverrideSpeed);
                if(gamepad2.dpadUpWasPressed()){
                    pollenTurretHood.setPosition(pollenTurretHood.getPosition() + 0.1);
                }
                else if(gamepad2.dpadDownWasPressed()){
                    pollenTurretHood.setPosition(pollenTurretHood.getPosition() - 0.1);
                }

                if(gamepad2.left_stick_x > 0 && pollenTurretDrive.getCurrentPosition() > minTurretRot){
                    pollenTurretDrive.setPower(gamepad2.left_stick_x * -0.6);
                }
                else if(gamepad2.left_stick_x < 0 && pollenTurretDrive.getCurrentPosition() < maxTurretRot){
                    pollenTurretDrive.setPower(gamepad2.left_stick_x * -0.6 );
                }
                else{
                    pollenTurretDrive.setPower(0);
                }

                launcherOn = gamepad2.b;

                if(gamepad2.dpadRightWasPressed() && manualOverrideSpeed < 0.9){
                    manualOverrideSpeed -= 0.1;
                }
                else if(gamepad2.dpadLeftWasPressed() && manualOverrideSpeed > -0.9){
                    manualOverrideSpeed += 0.1;
                }

                if(launcherOn){
                    pollenTurretFlywheel.setPower(manualOverrideSpeed);
                    feedMotor.setPower(feedSpeed);
                }
                else{
                    pollenTurretFlywheel.setPower(0);
                }
            }

            /*
            if(gamepad1.dpadUpWasPressed() && hoodMax < 0.9){
                hoodMax += 0.1;
                hoodMin += 0.1;
            }
            else if(gamepad1.dpadDownWasPressed() && hoodMin > 0){
                hoodMax -= 0.1;
                hoodMin -= 0.1;
            }

            if(gamepad1.dpadRightWasPressed() && launcherMaxSpeed < -1000){
                launcherMinSpeed += 200;
                launcherMaxSpeed += 200;
            }
            else if(gamepad1.dpadLeftWasPressed() && launcherMinSpeed > -2800){
                launcherMinSpeed -= 200;
                launcherMaxSpeed -= 200;
            }
*/
            //Toggle intake
            if(gamepad1.right_bumper){

                intakeForwards = true;
                intakeBackwards = false;
            }
            else{
                intakeForwards = false;
            }

            if(gamepad1.left_bumper){
                intakeBackwards = true;
                intakeForwards = false;
            }
            else{
                intakeBackwards = false;
            }

            if(gamepad1.right_trigger_pressed){
                feedMotor.setPower(feedSpeed);
            }
            else if(gamepad1.left_trigger_pressed){
                feedMotor.setPower(-feedSpeed);
            }
            else{
                feedMotor.setPower(0);
            }

            if(gamepad1.xWasPressed()){
                targetCenter = !targetCenter;
            }

            if(gamepad2.startWasPressed() && !turretManualOverride){
                pollenTurretDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                pollenTurretDrive.setPower(0);
                pollenTurretFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                turretManualOverride = true;
            }

            //Enable/Disable intake system
            if(intakeForwards){
                intake.setPower(intakeSpeed);
            }
            else if(intakeBackwards){
                intake.setPower(-intakeSpeed);
            }
            else{
                intake.setPower(0);
            }

            //telemetry.addData("Position: ", localizer.pose());
            if(!turretManualOverride){
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
            }

            telemetry.addData("Pollen hood target: ", pollenTurretHood.getPosition());
            telemetry.addData("Pollen turret target: ", pollenTurretDrive.getTargetPosition());
            telemetry.addData("Pollen turret position: ", pollenTurretDrive.getCurrentPosition());

            telemetry.addLine("Pose: " + localizer.pose().x() + ", " + localizer.pose().y() + ", " + Math.toDegrees(localizer.pose().heading()));
            telemetry.addLine("Pollen turret speed range: " + launcherMinSpeed + " to " + launcherMaxSpeed);
            telemetry.addLine("Pollen turret hood range: " + hoodMin + " to " + hoodMax);
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
