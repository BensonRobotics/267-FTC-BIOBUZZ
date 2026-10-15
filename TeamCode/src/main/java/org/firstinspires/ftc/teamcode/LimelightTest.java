package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class LimelightTest extends LinearOpMode {
    Limelight3A limelight;
    public List<LimelightCapture> limelightCaptures;
    public int limelightCaptureIdx = 0;
    @Override
    public void runOpMode(){
        limelightCaptures = new ArrayList<>();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
        waitForStart();

        while(opModeIsActive()){
            LLResult result = limelight.getLatestResult();
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            telemetry.addLine("Current detected fiducials:");
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId(); // The ID number of the fiducial
                double x = fiducial.getTargetXDegrees(); // Where it is (left-right)
                double y = fiducial.getTargetYDegrees(); // Where it is (up-down)
                double distance = calculateDistance(fiducial.getRobotPoseTargetSpace().getPosition().x, fiducial.getRobotPoseTargetSpace().getPosition().y, fiducial.getRobotPoseTargetSpace().getPosition().z);

                telemetry.addData("Fiducial id: ", id);
                telemetry.addData("x: ", x);
                telemetry.addData("y: ", y);
                telemetry.addData("distance: ", distance);
            }
            if(fiducials.toArray().length == 0){
                telemetry.addLine("No april tags detected");
            }
            telemetry.addData("Current capture position ", limelightCaptureIdx);
            if(gamepad1.aWasPressed()){
                limelightCaptures.add(new LimelightCapture(fiducials, limelightCaptureIdx));
                limelightCaptureIdx++;
            }
            telemetry.addLine("Previously captured positions:");

                for(LimelightCapture capture : limelightCaptures){
                    telemetry.addLine("");
                    telemetry.addData("Capture: ", capture.positionIdx);
                    for(LLResultTypes.FiducialResult fiducial : capture.fiducials){
                        int id = fiducial.getFiducialId(); // The ID number of the fiducial
                        double x = fiducial.getTargetXDegrees(); // Where it is (left-right)
                        double y = fiducial.getTargetYDegrees(); // Where it is (up-down)
                        double distance = calculateDistance(fiducial.getRobotPoseTargetSpace().getPosition().x, fiducial.getRobotPoseTargetSpace().getPosition().y, fiducial.getRobotPoseTargetSpace().getPosition().z);

                        telemetry.addData("Fiducial id: ", id);
                        telemetry.addData("x: ", x);
                        telemetry.addData("y: ", y);
                        telemetry.addData("distance: ", distance);
                    }
                }

            telemetry.update();
        }
    }
    public static double calculateDistance(double x, double y, double z) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
    }
}
