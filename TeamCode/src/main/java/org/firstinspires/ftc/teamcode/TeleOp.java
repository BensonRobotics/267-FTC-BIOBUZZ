package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearLeftMotor;
    DcMotor rearRightMotor;
    GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode(){

        //Initialize motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "fl_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "fr_motor");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "rl_motor");
        rearRightMotor = hardwareMap.get(DcMotor.class, "rr_motor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        waitForStart();
        while(opModeIsActive()){
            pinpoint.update();
            double heading = pinpoint.getHeading(AngleUnit.RADIANS);

            double fr = 0;
            double fl = 0;
            double br = 0;
            double bl = 0;

            double leftStickY = gamepad1.left_stick_y;
            double leftStickX = gamepad1.left_stick_x;
            double rightStickX = -gamepad1.right_stick_x;
            double rightStickY = gamepad1.right_stick_y;

            double theta = Math.atan2(-leftStickX, leftStickY);
            theta += heading;

            telemetry.addData("IMU Yaw ", heading);

            double magnitude = Math.hypot(-leftStickX, leftStickY);

            double leftPower = leftStickY - leftStickX;
            double rightPower = -leftStickY - leftStickX;

            double rX = -gamepad1.right_stick_x;
            double flbr = magnitude * Math.sin(theta + (0.25 * Math.PI));
            double frbl = magnitude * Math.sin(theta - (0.25 * Math.PI));

            fr = frbl - rX;
            fl = flbr + rX;
            br = flbr - rX;
            bl = frbl + rX;

            double denominator = Math.max(Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(br), Math.abs(bl))), 1);

            fr /= -denominator;
            fl /= -denominator;
            br /= denominator;
            bl /= denominator;

            frontRightMotor.setPower(fr);
            frontLeftMotor.setPower(fl);
            rearRightMotor.setPower(br);
            rearLeftMotor.setPower(bl);
        }
    }
}
