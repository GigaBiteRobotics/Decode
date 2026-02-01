package org.firstinspires.ftc.teamcode.drive;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class MDOConstants {
    // Power level for the launcher mechanism (0.0 to 1.0)
    public static double launchPower = 0.8;

    // Enable or disable AprilTag vision processing for localization
    public static boolean useAprilTags = true;

    // Coordinates [x, y, z] for the red alliance target (e.g. goal)
    public static Double[] redTargetLocation = new Double[]{-70.0, 70.0, 40.0};

    // Coordinates [x, y, z] for the blue alliance target (e.g. goal)
    public static Double[] blueTargetLocation = new Double[]{-70.0, -70.0, 40.0};

    // Calibration constants for launcher trajectory calculations {velocity, gravity}
    public static Double[] launcherCalcConstants = new Double[]{280.00, 386.09};

    // Servo position for the lifter when raised
    public static Double LifterPositionHigh = 0.8;

    // Servo position for the lifter when lowered
    public static Double LifterPositionLow = 0.0;

    // Boolean array indicating which lifter motors/servos should be reversed
    public static boolean[] LifterReverseMap = new boolean[]{true, false, false};

    // Delay in milliseconds to wait for the lifter to reach the top position
    public static int LifterWaitToTopTimerMillis = 200;

    // Offset in degrees for the turret azimuth relative to the robot's IMU zero
    public static Double AzimuthIMUOffset = -80.0;

    // Enable correction of turret angle based on robot heading (IMU)
    public static boolean EnableTurretIMUCorrection = true;

    // Multiplier for azimuth control signals (e.g. gear ratio adjustment)
    public static Double AzimuthMultiplier = 1.0;

    // Master switch to enable or disable turret functionality
    public static boolean EnableTurret = true;

    // Allow the launcher calculator to automatically control the azimuth angle
    public static boolean EnableLauncherCalcAzimuth = true;

    // Mechanical offset for the elevation angle
    public static Double ElevationOffset = 0.0;

    // Multiplier for elevation control signals
    public static Double ElevationMultiplier = 1.0;

    // Minimum target distance allowed for elevation calculation (in Feet)
    public static Double ElevationMinIN = 2.0;

    // Maximum target distance allowed for elevation calculation (in Feet)
    public static Double ElevationMaxIN = 14.0;

    // Minimum physical servo position for elevation
    public static Double ElevationMin = -0.4;

    // Maximum physical servo position for elevation
    public static Double ElevationMax = 0.9;

    // Offset applied to handle azimuth wrap-around logic
    public static Double AzimuthWrapAroundOffset = 0.0;
    // PIDF constants for controlling the azimuth servos {kP, kI, kD, kF}
    public static double[] AzimuthPIDFConstants = new double[]{4.0, 0.2, 0.0, 0.0};
    public static double[] CameraOffset = new double[]{-4.2, -7.5, -12.7}; // {x, y, z} offsets of the camera from robot center in inches
    // Maximum distance from an AprilTag to use it for localization (in inches)
    public static double AprilTagMaxDistance = 60.0;
}
