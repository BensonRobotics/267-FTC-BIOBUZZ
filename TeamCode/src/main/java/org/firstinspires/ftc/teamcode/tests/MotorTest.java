package org.firstinspires.ftc.teamcode.tests;

import static com.pedropathing.utils.Utils.lerp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class MotorTest extends LinearOpMode {
    public DcMotorEx testMotor;
    @Override
    public void runOpMode(){
        testMotor = hardwareMap.get(DcMotorEx.class, "test");
        waitForStart();
        while(opModeIsActive()){
            testMotor.setVelocity(2800);

            telemetry.addData("TestMotor Vel:", testMotor.getVelocity());
            telemetry.update();
        }
    }
}
