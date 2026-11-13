package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

public class Configuration {
    public static Alliance alliance = Alliance.Blue;
    public static StartingPos startingPos;
    public static Pose endingPos;
    public static double turretWireWrapAmount;
    public static AutoMode autoMode;

    //TODO: Tune
    public static final int maxTurretTickCount = 20;
    public static final int minTurretTickCount = -20;
}
