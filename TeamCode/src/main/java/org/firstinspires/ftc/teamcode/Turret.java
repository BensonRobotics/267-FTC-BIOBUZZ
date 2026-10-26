package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.LauncherProto.lerp;
import static org.firstinspires.ftc.teamcode.TeleOp.calculateDistance;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDController;

import java.util.List;

@Autonomous
public class Turret extends LinearOpMode {
    DcMotor flywheel;
    Servo hood;
    Servo servo;
    Limelight3A limelight;
    //PID constants
    //TODO: tune
    final double p = 0.01d;
    final double i = 0.0d;
    final double d = 0.001d;

    final double defaultHoodPosition = 0.5;

    final double maxDistance = 1.6;
    final double hoodMin = 0.2;
    final double hoodMax = 0.6;

    final double launcherMinSpeed = 0.1;
    final double launcherMaxSpeed = 1;

    //PID controller for pollen
    PIDController PID;
    @Override
    public void runOpMode(){

        PID = new PIDController(p, i, d);

        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        hood = hardwareMap.get(Servo.class, "hood");
        servo = hardwareMap.get(Servo.class, "servo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        limelight.pipelineSwitch(0);
        waitForStart();
        hood.setPosition(defaultHoodPosition);

        while(opModeIsActive()){
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
                servo.setPosition(defaultHoodPosition);
            }
            else{

                telemetry.addData("Targeting tag: ", closestTag.getFiducialId());

                //Rough estimates for flywheel power & hood position
                hood.setPosition(lerp(hoodMin, hoodMax, maxDistance/closestTagDistance));

                flywheel.setPower(lerp(launcherMinSpeed, launcherMaxSpeed, maxDistance/closestTagDistance));

                //PID input for targeting
                PID.setSetPoint(0.0);
                double correction = PID.calculate(closestTag.getTargetXPixels());

                telemetry.addData("Pollen PID correction: ", correction);

                servo.setPosition(correction);
            }
            telemetry.update();
        }
    }
}
