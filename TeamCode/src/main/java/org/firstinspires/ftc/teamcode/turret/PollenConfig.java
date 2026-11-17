package org.firstinspires.ftc.teamcode.turret;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PollenConfig {
    static final String flywheel = "pollen_launcher";
    static final String driveServo = "pollen_turret_drive";
    static final String hoodServo = "pollen_hood";
    static final String driveServoFourthWire = "pollen_turret_fourth_wire";
    static final String feedMotor = "feed_motor";
    static final double p = 1d;
    static final double i = 0.0d;
    static final double d = 0.0d;
    static final double hoodMax = 0.7;
    static final double hoodMin = 0.2;

    //Launcher settings
    static final double maxDistance = 1.1;

    //Speeds are in TPS (Ticks Per Seconds) counts
    static final double launcherMinSpeed = -2200;
    static final double launcherMaxSpeed = -1700;

    static final double maxTurretRot = 1;
    static final double minTurretRot = 0;
    public static Turret CreateConfig(HardwareMap hwMap, Telemetry tel){
        return new Turret(hwMap.get(DcMotorEx.class, flywheel), hwMap.get(DcMotor.class, feedMotor), hwMap.get(Servo.class, driveServo),
                hwMap.get(AnalogInput.class, driveServoFourthWire), hwMap.get(Servo.class, hoodServo),
                new PIDController(p, i, d), tel, hoodMax, hoodMin, maxDistance, launcherMinSpeed,
                launcherMaxSpeed, maxTurretRot, minTurretRot);
    }
}
