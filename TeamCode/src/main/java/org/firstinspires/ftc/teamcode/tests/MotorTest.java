package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MotorTest extends LinearOpMode {
    public DcMotor testMotor;
    @Override
    public void runOpMode(){
        testMotor = hardwareMap.get(DcMotor.class, "test");
        waitForStart();
        while(opModeIsActive()){
            testMotor.setPower(1);
        }
    }
}
