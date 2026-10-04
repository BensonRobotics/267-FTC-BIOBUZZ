package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
public class IntakeProto extends LinearOpMode {
    DcMotor motor;
    double speed = 1;
    @Override
    public void runOpMode(){
        motor = hardwareMap.get(DcMotor.class, "intakeMotor");
        waitForStart();
        while(opModeIsActive()){
            motor.setPower(speed);
        }
    }
}
