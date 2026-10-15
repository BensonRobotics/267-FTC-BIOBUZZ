package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class LauncherProto extends LinearOpMode {
    public DcMotorEx launchMotor;
    public Servo hoodServo;
    public double hoodServoTarget = 0.0d;
    public double launchMotorPower = 0.0d;
    public Limelight3A limelight;

    @Override
    public void runOpMode(){
        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");


        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.dpadUpWasPressed() && hoodServoTarget < 1.0){
                hoodServoTarget+=0.1d;
            }
            else if(gamepad1.dpadDownWasPressed() && hoodServoTarget > 0.0){

                hoodServoTarget-=0.1d;
            }

            if(gamepad1.dpadRightWasPressed() && launchMotorPower < 1.0){
                launchMotorPower += 0.1d;
            }
            else if(gamepad1.dpadLeftWasPressed() && launchMotorPower > -1.0){
                launchMotorPower -= 0.1d;
            }


            hoodServo.setPosition(hoodServoTarget);
            launchMotor.setPower(launchMotorPower);

            if(gamepad1.aWasPressed()){
                launchMotor.setPower(0);
                launchMotorPower = 0.0d;
            }

            telemetry.addData("ServoPos: ",hoodServo.getPosition());
            telemetry.addData("MotorSpeed: ",launchMotor.getPower());
            telemetry.update();
        }
    }
}
