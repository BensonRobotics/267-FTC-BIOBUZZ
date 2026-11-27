package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.obselete.LauncherProto.lerp;

import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.comp_opmodes.Auto;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.sql.Time;
import java.time.LocalTime;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Turret Tuner", group = "Tune")
public class TurretTuner extends LinearOpMode {
    DcMotor intake;
    DcMotor feedMotor;

    DcMotorEx pollenTurretFlywheel;
    DcMotorEx pollenTurretDrive;
    Servo pollenTurretHood;
    boolean intakeForwards;
    boolean launcherOn;
    boolean intakeBackwards;

    double intakeSpeed = -0.75;
    double feedSpeed = -0.8;

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

    final double launcherVelocityTolerance = 50;
    double launcherVelocityTarget = 0;

    Limelight3A limelight;

    //PID constants
    //TODO: tune

    public static volatile double P = 2d;
    public static volatile double I = 0.0d;
    public static volatile double D = 0.0d;

    //PID controller for pollen
    PIDController pollenPID;
    boolean feedAllowed = true;
    public double powerCompensation;

    @Override
    public void runOpMode(){
        pollenTurretDrive = hardwareMap.get(DcMotorEx.class, "pollen_turret_drive");


        //Initialize pollen PID controller
        pollenPID = new PIDController(P, I, D);

        //Initialize intake system
        intake = hardwareMap.get(DcMotor.class, "intake_motor");
        feedMotor = hardwareMap.get(DcMotor.class, "feed_motor");

        //Initialize pollen turret system
        pollenTurretFlywheel = hardwareMap.get(DcMotorEx.class, "pollen_launcher");
        pollenTurretHood = hardwareMap.get(Servo.class, "pollen_hood");

        feedMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Initialize limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        limelight.pipelineSwitch(0);
        VoltageSensor ControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        waitForStart();

        pollenTurretDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pollenTurretDrive.setTargetPosition(0);
        pollenTurretDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pollenTurretDrive.setPower(1);

        //Predefine pollen hood position
        pollenTurretHood.setPosition(0.5);


        MenuHandler menu = new MenuHandler(telemetry, new MenuOption[]
                {
                        new MenuOption("P", 0.5, 0, 15, 6),
                        new MenuOption("I", 0.05, 0, 15, 0.15),
                        new MenuOption("D", 0.05, 0, 15, 0.05),
                        new MenuOption("Turret Max Speed", 100, -2800, -500, -1600),
                        new MenuOption("Turret Min Speed", 100, -2800, -500, -1500),
                        new MenuOption("Turret Hood Min", 0.05, 0, 1, 0.6),
                        new MenuOption("Turret Hood Max", 0.05, 0, 1, 0.8),
                        new MenuOption("Intake Speed", 0.1, 0, -1, -0.75),
                        new MenuOption("Feed Speed", 0.1, 0, -1, -0.8)
                });


        //Main event loop
        while(opModeIsActive()){
            //dashboard.updateConfig();
            powerCompensation = 13 / ControlHubVoltageSensor.getVoltage();

            menu.Redraw();
            if (gamepad1.dpadDownWasPressed() ) {
                menu.ChangeSelection(1);
            } else if (gamepad1.dpadUpWasPressed() ) {
                menu.ChangeSelection(-1);
            } else if (gamepad1.dpadRightWasPressed() ) {
                menu.EditCurrentSelection(1);
            }else if(gamepad1.dpadLeftWasPressed()){
                menu.EditCurrentSelection(-1);
            }

            MenuOptionExport[] options = menu.ExportSelections();
            if((P != options[0].exportedNumber) || (I != options[1].exportedNumber) || (D != options[2].exportedNumber)){
                P = options[0].exportedNumber;
                I = options[1].exportedNumber;
                D = options[2].exportedNumber;
                pollenPID = new PIDController(P, I, D);

            }

            launcherMaxSpeed = options[3].exportedNumber;
            launcherMinSpeed = options[4].exportedNumber;
            hoodMin = options[5].exportedNumber;
            hoodMax = options[6].exportedNumber;
            intakeSpeed = options[7].exportedNumber;
            feedSpeed = options[8].exportedNumber;

            //Fetch limelight
            LLResult result = limelight.getLatestResult();

            //Check for limelight disconnect and warn user
            if(!limelight.isConnected()){
                telemetry.addLine("WARN: Limelight disconnected. Robot reboot recommended. Check wiring.");
            }

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



                if (closestTag == null) {
                    launcherVelocityTarget = 0;
                    feedAllowed = true;
                    pollenPID.reset();
                } else {
                    telemetry.addData("Targeting tag: ", closestTag.getFiducialId());

                    //Rough estimates for flywheel power & hood position
                    pollenTurretHood.setPosition(lerp(hoodMax, hoodMin, closestTagDistance / maxDistance));
                    //Toggle launcher hardware
                    if (launcherOn) {
                        launcherVelocityTarget = lerp(launcherMaxSpeed, launcherMinSpeed, closestTagDistance / maxDistance);
                        if (Math.abs(pollenTurretFlywheel.getVelocity() - launcherVelocityTarget) < launcherVelocityTolerance) {
                            feedAllowed = true;
                        }
                        else{
                            feedAllowed = false;
                        }
                    } else {
                        launcherVelocityTarget = 0;
                        feedAllowed = true;
                    }

                    //PID input for targeting
                    pollenPID.setSetPoint(0.0);
                    double correction = pollenPID.calculate(closestTag.getTargetXDegrees());
                    telemetry.addData("Correction: ", correction);
                    pollenTurretDrive.setTargetPosition((int) Math.round(Range.clip(pollenTurretDrive.getCurrentPosition() + (correction * (537.7 / 360)), minTurretRot, maxTurretRot)));
                }
                if(gamepad1.b){
                    launcherOn = true;
                }
                else{
                    launcherOn = false;
                }
                pollenTurretFlywheel.setVelocity(launcherVelocityTarget);

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



            if(gamepad1.right_trigger_pressed & feedAllowed){
                feedMotor.setPower(feedSpeed * powerCompensation);
            }
            else if(gamepad1.left_trigger_pressed){
                feedMotor.setPower(-feedSpeed * powerCompensation);
            }
            else{
                feedMotor.setPower(0);
            }

            //Enable/Disable intake system
            if(intakeForwards){
                intake.setPower(intakeSpeed * powerCompensation);
            }
            else if(intakeBackwards){
                intake.setPower(-intakeSpeed * powerCompensation);
            }
            else{
                intake.setPower(0);
            }
            telemetry.addLine("Launcher vel: " + pollenTurretFlywheel.getVelocity() + "/" + launcherVelocityTarget);
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
