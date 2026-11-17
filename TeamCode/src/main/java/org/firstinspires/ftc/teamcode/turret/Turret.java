package org.firstinspires.ftc.teamcode.turret;

import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.obselete.LauncherProto.lerp;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class Turret {
    double hoodMax;
    double hoodMin;

    //Launcher settings
    double maxDistance;

    //Speeds are in TPS (Ticks Per Seconds) counts
    double launcherMinSpeed;
    double launcherMaxSpeed;

    //Between 0 and 1
    double maxTurretRot;
    double minTurretRot;

    DcMotorEx flywheel;
    Servo turret;
    AnalogInput turretFourthWire;
    Servo hood;

    //No clue why I added this
    DcMotor feedMotor;
    PIDController PID;
    Telemetry telemetry;

    public Turret(DcMotorEx flywheelInput, DcMotor feedMotorInput, Servo turretInput, AnalogInput turretFourthWireInput, Servo hoodInput, PIDController pid, Telemetry telemetryInput,
                  double hoodMaxInput, double hoodMinInput, double maxDistanceInput, double launcherMinSpeedInput,
                  double launcherMaxSpeedInput, double maxTurretRotInput, double minTurretRotInput){
        flywheel = flywheelInput;
        feedMotor = feedMotorInput;
        turret = turretInput;
        turretFourthWire = turretFourthWireInput;
        hood = hoodInput;
        PID = pid;
        telemetry = telemetryInput;
        hoodMax = hoodMaxInput;
        hoodMin = hoodMinInput;
        maxDistance = maxDistanceInput;
        launcherMinSpeed = launcherMinSpeedInput;
        launcherMaxSpeed = launcherMaxSpeedInput;
        maxTurretRot = maxTurretRotInput;
        minTurretRot = minTurretRotInput;
    }

    public void FindTag(List<LLResultTypes.FiducialResult> tags, boolean launcherOn, double heading){
        LLResultTypes.FiducialResult closestTag = null;
        double closestTagDistance = Double.POSITIVE_INFINITY;

        //Find closest april tag
        for (LLResultTypes.FiducialResult fiducial : tags) {
            Pose3D robotPose = fiducial.getRobotPoseTargetSpace();


            double distance = -robotPose.getPosition().z;
            if (distance < closestTagDistance) {
                closestTagDistance = distance;
                closestTag = fiducial;
            }
        }
        double launcherVelocityTarget = 0;
        //Check for any tags
        if (closestTag == null) {
                double bearing = Math.toDegrees(Math.atan2(70 -heading, 70 - heading));
                double rel = AngleUnit.normalizeDegrees(bearing - Math.toDegrees(heading));
                turret.setPosition((int) Math.round(clip(rel, minTurretRot, maxTurretRot)));
                PID.reset();
        } else {
            telemetry.addData("Targeting tag: ", closestTag.getFiducialId());
            hood.setPosition(lerp(hoodMax, hoodMin, closestTagDistance / maxDistance));
            //Toggle launcher hardware
            if (launcherOn) {
                launcherVelocityTarget = lerp(launcherMaxSpeed, launcherMinSpeed, closestTagDistance / maxDistance);
                //Only enable feed when pollen flywheel is up to speed
                //if (Math.abs(pollenTurretFlywheel.getVelocity() - launcherVelocityTarget) < launcherVelocityTolerance) {
                //    feedMotor.setPower(feedSpeed);
                //}
            } else {
                launcherVelocityTarget = 0;
            }

            //PID input for targeting
            PID.setSetPoint(0.0);
            double correction = PID.calculate(closestTag.getTargetXDegrees());
            turret.setPosition((int) Math.round(Range.clip((turretFourthWire.getVoltage() / 3.3) + correction, minTurretRot, maxTurretRot)));
        }
        flywheel.setVelocity(launcherVelocityTarget);
    }
}
