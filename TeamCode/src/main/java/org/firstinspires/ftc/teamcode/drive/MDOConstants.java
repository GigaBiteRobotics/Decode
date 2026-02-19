package org.firstinspires.ftc.teamcode.drive;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class MDOConstants {
    public static double AprilTagMaxDistance = 60.0;
    public static int AprilTagUpdateIntervalMs = 100;
    public static Double AzimuthFineAdjustment = 0.0;
    public static double AzimuthForbiddenZoneCenter = 50;
    public static double AzimuthForbiddenZoneWidth = 0.0; // Total width of forbidden zone in degrees (half on each side of center)
    public static double AzimuthForbiddenZoneEscapePower = 0.5; // Power to use when escaping from inside forbidden zone
    public static Double AzimuthIMUOffset = 120.0;
    public static Double AzimuthMultiplier = 1.0;
    public static double AzimuthPIDDeadzonePercent = 0.05;
    public static double[] AzimuthPIDFConstants = new double[]{4.0, 0.2, 0.0, 0.0};
    public static double AzimuthServoCenterOffset = 0.0;
    public static double BlueAprilTagHeadingOffset = 90.0;
    public static Double BlueAzimuthFineAdjustment = 0.0;
    public static double[] BlueCloseStartPose = new double[]{22.85, 124.85, -40.7};
    public static double[] BlueFarStartPose = new double[]{72.0, 72.0, -90.0};
    public static Double[] BlueLauncherCalcConstants = new Double[]{280.00, 386.09};
    public static int BlueLauncherRPM = 2300;
    public static double[][] BlueLauncherRPMZones = new double[][]{
            {0.0, 4000},
            {110.0, 6000},
    };
	public static double[][] BlueElevationOffsetZones = new double[][]{
			{0.0, 0},
			{110, -0.5},
	};
    public static Double[] blueTargetLocation = new Double[]{2.0, 142.0, 40.0};
	public static Pose blueCloseLaunchLocation = new Pose(65, 80, -1.8);
	public static Pose blueHumanLocation = new Pose(144-39, 16, 0);
	public static Pose blueGateLocation = new Pose(27,65,-Math.PI);
    public static double[] CameraOffset = new double[]{-4.2, -7.5, -12.7};
    public static double ColorMinBrightness = 0.0;
    public static int ColorMinRawSum = 5;
    public static double ColorMinSaturation = 0.10;
    public static int[] ColorSensorPitMapping = new int[]{0, 1, 4, 5, 2, 3};
    public static Double ElevationMax = 0.9;
    public static Double ElevationMaxIN = 14.0;
    public static Double ElevationMin = -0.4;
    public static Double ElevationMinIN = 2.0;
    public static Double ElevationMultiplier = 1.0;
    public static Double ElevationOffset = 0.0;
    public static Double RedElevationOffset = 0.0;
    public static Double BlueElevationOffset = 0.0;
    public static double EmergencyStopTemp = 88.0;
    public static boolean EnableFieldCentricDrive = true;
    public static boolean EnableLauncherCalcAzimuth = true;
    public static boolean EnableLauncherRPMZones = true;
    public static boolean EnableThreadedDrive = true;
    public static boolean EnableThreadedFollowerUpdate = true;
    public static boolean EnableTurret = true;
    public static boolean EnableTurretIMUCorrection = true;
    public static double GreenHueMax = 190.0;
    public static double GreenHueMin = 80.0;
    public static CustomPIDFController LauncherPIDF = new CustomPIDFController(20, 2, 0, 3);
    public static boolean EnableLauncherPID = true;
    public static double LauncherManualPower = 1.0;
    public static double launchPower = 1;
    public static Double LifterPositionHigh = 0.8;
    public static Double LifterPositionLow = 0.0;
    public static int[] LifterPitMapping = new int[]{0, 1, 2};
    public static boolean[] LifterReverseMap = new boolean[]{true, false, false};
    public static int LifterWaitToTopTimerMillis = 200;
    public static double PurpleHueMax = 330.0;
    public static double PurpleHueMin = 200.0;
    public static int RapidFireDelayMs = 500;
    public static int[] RapidFireOrder = new int[]{0, 1, 2};
    public static double RedAprilTagHeadingOffset = 90.0;
    public static Double RedAzimuthFineAdjustment = 0.0;
    public static double[] RedCloseStartPose = new double[]{121.15, 124.85, -130.7};
    public static double[] RedFarStartPose = new double[]{72.0, 72.0, -90.0};
    public static Double[] RedLauncherCalcConstants = new Double[]{280.00, 386.09};
    public static int RedLauncherRPM = 2300;
    public static double[][] RedLauncherRPMZones = new double[][]{
            {0.0, 3400},
            {110.0, 4200},
    };
	public static double[][] RedElevationOffsetZones = new double[][]{
			{0.0, 0},
			{110, -0.5},
	};
    public static Double[] redTargetLocation = new Double[]{142.0, 142.0, 40.0};
    public static Pose redCloseLaunchLocation = new Pose(89, 90, Math.PI - 1.31);
    public static Pose redHumanLocation = new Pose(144, 0, Math.PI);
	public static Pose redGateLocation = new Pose(144-27,65,0);
    public static boolean useAprilTags = true;
}
