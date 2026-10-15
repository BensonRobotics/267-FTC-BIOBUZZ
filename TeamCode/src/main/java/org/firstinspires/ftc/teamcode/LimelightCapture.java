package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

public class LimelightCapture {
    public List<LLResultTypes.FiducialResult> fiducials;
    public int positionIdx;


    public LimelightCapture(List<LLResultTypes.FiducialResult> inputF, int inputP){
        this.fiducials = inputF;
        this.positionIdx = inputP;
    }
}
