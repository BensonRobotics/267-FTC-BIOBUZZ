package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.obselete.LauncherProto.lerp;
import static org.firstinspires.ftc.teamcode.comp_opmodes.TeleOp.calculateDistance;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDController;

import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@Autonomous
public class Turret extends LinearOpMode {
    DcMotor flywheel;
    Servo hood;
    DcMotorEx turret;
    Limelight3A limelight;
    //PID constants
    //TODO: tune
    final double p = 0.0001d;
    final double i = 0d;
    final double d = 0d;

    final double defaultHoodPosition = 0.5;

    final double maxDistance = 1.6;
    final double hoodMin = 0.2;
    final double hoodMax = 0.6;

    final double launcherMinSpeed = 0.1;
    final double launcherMaxSpeed = 1;

    PIDController PID;


    @Override
    public void runOpMode(){
        PID.setPID(p, i, d);
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        hood = hardwareMap.get(Servo.class, "hood");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        limelight.pipelineSwitch(0);
        waitForStart();

        ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1);
        Runnable task = this::AcquireLimelightTarget;
        long initialDelay = 0;
        long period = 5;

        scheduler.scheduleWithFixedDelay(task, initialDelay, period, TimeUnit.MILLISECONDS);
        hood.setPosition(defaultHoodPosition);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(opModeIsActive()){

            telemetry.update();
        }
    }

    public void AcquireLimelightTarget(){
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
            hood.setPosition(defaultHoodPosition);
        }
        else{
            telemetry.addData("Targeting tag num: ", closestTag.getFiducialId());
            hood.setPosition(lerp(hoodMin, hoodMax, maxDistance/closestTagDistance));
            PID.setSetPoint(0);
            double correction = PID.calculate(closestTag.getTargetXDegrees());
            telemetry.addData("Pollen PID correction: ", correction);
            telemetry.addData("Target X pixels: ", closestTag.getTargetXDegrees());
            telemetry.addData("Target distance: ", closestTagDistance);
            turret.setTargetPosition((int)Math.round(turret.getCurrentPosition() + correction));
        }
    }
}
