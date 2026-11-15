package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

public class Configuration {
    public static Alliance alliance = Alliance.Blue;
    public static StartingPos startingPos = StartingPos.LeftOfPark;
    public static Pose endingPos = null;
    public static double turretWireWrapAmount;
    public static AutoMode autoMode = AutoMode.JustShoot;

    //TODO: Tune
    public static final int maxTurretTickCount = 20;
    public static final int minTurretTickCount = -20;
}
