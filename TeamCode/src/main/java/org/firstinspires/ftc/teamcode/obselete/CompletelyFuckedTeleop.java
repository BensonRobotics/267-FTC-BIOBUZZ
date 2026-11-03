package org.firstinspires.ftc.teamcode.obselete;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDController;

@Deprecated
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Fucked teleop", group = "Fucked")
public class CompletelyFuckedTeleop extends LinearOpMode {
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearLeftMotor;
    DcMotor rearRightMotor;
    DcMotor intake;
    DcMotor feedMotor;

    DcMotor pollenTurretFlywheel;
    Servo pollenTurretHood;
    Servo pollenTurretServo1;
    //Servo pollenTurretServo2;
    Localizer localizer;

    //DcMotor nectarTurretFlywheel;
    //Servo nectarTurretHood;
    //Servo nectarTurretServo;
    GoBildaPinpointDriver pinpoint;
    int intakeOn;
    boolean launcherOn;
    double launchSpeed;
    double hoodAngle = .2;
    Limelight3A limelight;

    //PID constants
    //TODO: tune
    final double p = 0.01d;
    final double i = 0.0d;
    final double d = 0.001d;

    double pollenTurretAngle = 0;

    //PID controller for pollen
    PIDController pollenPID;

    public Pose2D startingPosition;

    Follower follower;

    IMU imu;
    @Override
    public void runOpMode(){
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        follower = Constants.create(hardwareMap, localizer);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        //Initialize pollen PID controller
        pollenPID = new PIDController(p, i, d);

        //Initialize drive motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "fl_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "fr_motor");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "rl_motor");
        rearRightMotor = hardwareMap.get(DcMotor.class, "rr_motor");

        //Initialize intake system
        intake = hardwareMap.get(DcMotor.class, "intake_motor");
        feedMotor = hardwareMap.get(DcMotor.class, "feed_motor");

        //Initialize pollen turret system
        pollenTurretFlywheel = hardwareMap.get(DcMotor.class, "pollen_launcher");
        pollenTurretHood = hardwareMap.get(Servo.class, "pollen_hood");
        pollenTurretServo1 = hardwareMap.get(Servo.class, "pollen_servo");
        //pollenTurretServo2 = hardwareMap.get(Servo.class, "pollen_servo_2");


        //Nectar not used for first version
        //nectarTurretFlywheel = hardwareMap.get(DcMotor.class, "nectar_launcher");
        //nectarTurretHood = hardwareMap.get(Servo.class, "nectar_hood");
        //nectarTurretServo = hardwareMap.get(Servo.class, "nectar_servo");

        //Initialize pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        //pinpoint.setPosition(startingPosition);

        //Initialize limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        //Swap limelight to pipeline 0 (Blue I believe)
        limelight.pipelineSwitch(0);

        waitForStart();

        //Predefine pollen hood position
        pollenTurretHood.setPosition(0.5);

        //Main event loop
        while(opModeIsActive()){

            //Update pinpoint odometry
            pinpoint.update();

            localizer.update();

            //Fetch limelight
//            LLResult result = limelight.getLatestResult();
//
//            //Check for limelight disconnect and warn user
//            if(!limelight.isConnected()){
//                telemetry.addLine("Limelight disconnected. Robot reboot recommended. Check wiring.");
//            }
//
//            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
//
//            //Init variables for closest tag
//            LLResultTypes.FiducialResult closestTag = null;
//            double closestTagDistance = Double.POSITIVE_INFINITY;
//
//            //Find closest april tag
//            for (LLResultTypes.FiducialResult fiducial : fiducials) {
//                double distance = calculateDistance(fiducial.getRobotPoseTargetSpace().getPosition().x, fiducial.getRobotPoseTargetSpace().getPosition().y, fiducial.getRobotPoseTargetSpace().getPosition().z);
//                if(distance < closestTagDistance){
//                    closestTagDistance = distance;
//                    closestTag = fiducial;
//                }
//            }
//
//            //Check for any tags
//            if(closestTag == null){
//                telemetry.addLine("No tags detected");
//
//                //Return servo hood to midpoint
//                pollenTurretServo1.setPosition(servoHoodDefault);
//                //pollenTurretServo2.setPosition(servoHoodDefault);
//            }
//            else{
//
//                telemetry.addData("Targeting tag: ", closestTag.getFiducialId());
//
//                //Rough estimates for flywheel power & hood position
//                pollenTurretHood.setPosition(lerp(hoodMin, hoodMax, maxDistance/closestTagDistance));
//
//                //Toggle launcher hardware
//                if(launcherOn){
//                    pollenTurretFlywheel.setPower(lerp(launcherMinSpeed, launcherMaxSpeed, maxDistance/closestTagDistance));
//                }
//                else{
//                    pollenTurretFlywheel.setPower(0);
//                }
//
//                //PID input for targeting
//                pollenPID.setSetPoint(0.0);
//                double correction = pollenPID.calculate(closestTag.getTargetXPixels());
//
//                telemetry.addData("Pollen PID correction: ", correction);
//
//                pollenTurretServo1.setPosition(correction);
//                //pollenTurretServo2.setPosition(correction);
//            }

            telemetry.addData("Pinpoint pos: ", localizer.pose());

            //Toggle intake
            if(gamepad1.aWasPressed()){
                launcherOn = !launcherOn;
            }

            if(launcherOn){
                pollenTurretFlywheel.setPower(launchSpeed);
            }
            else{
                pollenTurretFlywheel.setPower(0);
            }

            if(gamepad1.bWasPressed()){
                if(intakeOn == 1){
                    intakeOn = 0;
                }
                else{
                    intakeOn = -1;
                }
            }
            else if(gamepad1.xWasPressed()){
                if(intakeOn == 1){
                    intakeOn = 0;
                }
                else{
                    intakeOn = 1;
                }
            }

            if(gamepad1.yWasPressed()){
                follower.setHeading(0);
            }

            intake.setPower(intakeOn);
            feedMotor.setPower(-intakeOn * 0.8);

            if(gamepad1.dpadUpWasPressed() && launchSpeed < 0.9){
                launchSpeed+= 0.1;
            }
            else if(gamepad1.dpadDownWasPressed() && launchSpeed > -0.9){
                launchSpeed-=0.1;
            }


            if(gamepad1.dpadRightWasPressed() && pollenTurretAngle < 0.9){
                pollenTurretAngle += 0.05;
            }
            else if(gamepad1.dpadLeftWasPressed() && pollenTurretAngle > -0.9){
                pollenTurretAngle -= 0.05;
            }

            pollenTurretServo1.setPosition(pollenTurretAngle);

            if(gamepad1.leftBumperWasPressed() && hoodAngle > -0.9){
                hoodAngle -= 0.05;
            }
            else if(gamepad1.rightBumperWasPressed() && hoodAngle < 0.9){
                hoodAngle += 0.05;
            }

            pollenTurretHood.setPosition(hoodAngle);


            telemetry.addData("Launch speed: ", launchSpeed);
            telemetry.addData("Launcher stat: ", launcherOn);
            telemetry.addData("Hood angle: ", hoodAngle);
            telemetry.addData("Turret angle: ", pollenTurretServo1.getPosition());
            telemetry.addData("Intake speed: ", intakeOn);

            //Some really cool movement code I stole from bartholomew
            //double heading = pinpoint.getHeading(AngleUnit.RADIANS);
            double heading = localizer.pose().heading();

//            double fr = 0;
//            double fl = 0;
//            double br = 0;
//            double bl = 0;
//
//            double leftStickY = gamepad1.left_stick_y;
//            double leftStickX = gamepad1.left_stick_x;
//            double rightStickX = -gamepad1.right_stick_x;
//            double rightStickY = gamepad1.right_stick_y;
//
//            double theta = Math.atan2(-leftStickX, leftStickY);
//            theta += heading;
//
//            telemetry.addData("IMU Yaw ", heading);
//
//            double magnitude = Math.hypot(-leftStickX, leftStickY);
//
//            double leftPower = leftStickY - leftStickX;
//            double rightPower = -leftStickY - leftStickX;
//
//            double rX = -gamepad1.right_stick_x;
//            double flbr = magnitude * Math.sin(theta + (0.25 * Math.PI));
//            double frbl = magnitude * Math.sin(theta - (0.25 * Math.PI));
//
//            fr = frbl - rX;
//            fl = flbr - rX;
//            br = flbr + rX;
//            bl = frbl + rX;
//
//            double denominator = Math.max(Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(br), Math.abs(bl))), 1);
//
//            fr /= -denominator;
//            fl /= -denominator;
//            br /= denominator;
//            bl /= denominator;
//
//            frontRightMotor.setPower(fr);
//            frontLeftMotor.setPower(fl);
//            rearRightMotor.setPower(br);
//            rearLeftMotor.setPower(bl);

//            DrivePowers powers = ManualDrive.fieldCentric(
//                    -gamepad1.left_stick_y,
//                    gamepad1.left_stick_x,
//                    gamepad1.right_stick_x,
//                    imu.getHeading
//            );
            //follower.manual(powers);
            follower.update();
            telemetry.update();
        }
    }

    public double Interpolate(double min, double max, double in){
        return min + in * (max - in);
    }

    public static double calculateDistance(double x, double y, double z) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
    }
}
