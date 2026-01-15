package org.firstinspires.ftc.teamcode.drive;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class MDOConstants {
    public static boolean usePIDFLauncher = false;
    public static boolean useAprilTags = true;
    public static Double[] targetLocation = new Double[]{-70.0, 70.0, 40.0};
    public static CustomPIDFController launcherPIDF = new CustomPIDFController(1.5, 0.0, 0.2, 0.0);
    public static Double[] launcherCalcConstants = new Double[]{280.00, 386.09};
    public static double maxTurretAzimuthRotations = 1.5;
    public static double LifterPositionHigh = 0;
    public static double LifterPositionLow = 0;
    public static int LifterWaitToTopTimerMillis = 200;

    // ===== AIMING SYSTEM CONSTANTS =====
    // Enable/disable auto-aiming
    public static boolean enableAutoAiming = true;

    // Launcher height from ground (inches)
    public static double launcherHeightZ = 10.0;

    // Elevation servo tuning
    public static double elevationServoMin = 0.0;           // Servo position for minimum elevation
    public static double elevationServoMax = 1.0;           // Servo position for maximum elevation
    public static double elevationAngleMin = 0.0;           // Minimum elevation angle (radians)
    public static double elevationAngleMax = Math.toRadians(45.0); // Maximum elevation angle (radians)
    public static double elevationServoOffset = 0.0;        // Offset to calibrate servo center
    public static boolean elevationServoReversed = false;   // Reverse servo direction if needed

    // Azimuth servo tuning
    public static double azimuthServoMin = 0.0;             // Servo position for minimum azimuth
    public static double azimuthServoMax = 1.0;             // Servo position for maximum azimuth
    public static double azimuthAngleMin = -Math.PI;        // Minimum azimuth angle (radians)
    public static double azimuthAngleMax = Math.PI;         // Maximum azimuth angle (radians)
    public static double azimuthServoOffset = 0.0;          // Offset to calibrate servo center
    public static boolean azimuthServoReversed = false;     // Reverse servo direction if needed
    public static boolean useFieldRelativeAzimuth = true;   // Use field-relative aiming vs robot-relative

    // Dual servo configuration (if using two servos for azimuth)
    public static boolean useDualAzimuthServos = true;      // Enable second azimuth servo
    public static boolean azimuthServo1Mirrored = true;     // Mirror second servo (1.0 - position)

    // Aiming smoothing and limits
    public static double aimingSmoothingFactor = 0.2;       // 0.0 = no smoothing, 1.0 = instant
    public static double elevationDeadzone = Math.toRadians(1.0); // Ignore small elevation changes
    public static double azimuthDeadzone = Math.toRadians(2.0);   // Ignore small azimuth changes

    // Manual aiming adjustment
    public static double manualElevationStep = Math.toRadians(1.0);  // Adjustment per button press
    public static double manualAzimuthStep = Math.toRadians(2.0);    // Adjustment per button press
    public static double manualAdjustmentHoldTime = 100.0;           // ms between adjustments when held

    // Aiming validation
    public static double maxElevationError = Math.toRadians(5.0);  // Maximum acceptable elevation error
    public static double maxAzimuthError = Math.toRadians(5.0);    // Maximum acceptable azimuth error
    public static boolean requireAimLock = false;                  // Require aim lock before shooting
}
