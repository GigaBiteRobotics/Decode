package org.firstinspires.ftc.teamcode.drive;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class MDOConstants {
    public static boolean usePIDFLauncher = false;
    public static boolean useAprilTags = true;
    public static Double[] redTargetLocation = new Double[]{-70.0, 70.0, 40.0};
    public static Double[] blueTargetLocation = new Double[]{70.0, 70.0, 40.0};
    public static CustomPIDFController launcherPIDF = new CustomPIDFController(1.5, 0.0, 0.2, 0.0);
    public static Double[] launcherCalcConstants = new Double[]{280.00, 386.09};
    public static Double maxTurretAzimuthRotations = 1.5;
    public static Double LifterPositionHigh = 0.0;
    public static Double LifterPositionLow = 0.0;
    public static int LifterWaitToTopTimerMillis = 200;
    public static Double AzimuthIMUOffset = 0.0;
    public static boolean EnableTurretIMUCorrection = true;
    public static boolean UseBothAzimuthServos = true;
    public static Double AzimuthMultiplier = 1.0;
    public static boolean EnableTurret = true;
    public static boolean ReverseOneAzimuthServo = false;
    public static boolean EnableLauncherCalcAzimuth = true;
}
