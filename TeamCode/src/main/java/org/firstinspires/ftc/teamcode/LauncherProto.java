package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeleOp.calculateDistance;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@TeleOp
public class LauncherProto extends LinearOpMode {
    public DcMotorEx launchMotor;
    public Servo hoodServo;
    public double hoodServoTarget = 0.0d;
    public double launchMotorPower = 0.0d;
    public Limelight3A limelight;
    public boolean launcherOn;
    public double closestAprilTagDistance;

    @Override
    public void runOpMode(){
        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        waitForStart();
        while(opModeIsActive()){
            LLResult result = limelight.getLatestResult();
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
            }
            else{
                telemetry.addData("Targeting tag: ", closestTag.getFiducialId());

                hoodServoTarget = lerp(0, 0.5, 1.6/closestTagDistance);
                launchMotorPower = lerp(-0.8d, -0.1d, 1.6/closestTagDistance);
            }
            hoodServo.setPosition(hoodServoTarget);
            if(gamepad1.aWasPressed()){
                launcherOn = !launcherOn;
            }
            if(launcherOn){
                launchMotor.setPower(launchMotorPower);
            }
            telemetry.addData("ServoPos: ",hoodServo.getPosition());
            telemetry.addData("MotorSpeed: ",launchMotor.getPower());
            telemetry.update();
        }
    }
    public static double lerp(double start, double end, double t) {
        return start + t * (end - start);
    }

}
