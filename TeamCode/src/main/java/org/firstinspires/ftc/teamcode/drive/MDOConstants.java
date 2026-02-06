package org.firstinspires.ftc.teamcode.drive;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class MDOConstants {
    // Power level for the launcher mechanism (0.0 to 1.0)
    public static double launchPower = 0.8;

    // Enable or disable AprilTag vision processing for localization
    public static boolean useAprilTags = true;

    // Coordinates [x, y, z] for the red alliance target (e.g. goal)
    // Pedro Pathing coords: x_new = y_old + 72, y_new = -x_old + 72
    // Original (-70, 70, 40) → (142, 142, 40)
    public static Double[] redTargetLocation = new Double[]{142.0, 142.0, 40.0};

    // Coordinates [x, y, z] for the blue alliance target (e.g. goal)
    // Pedro Pathing coords: x_new = y_old + 72, y_new = -x_old + 72
    // Original (-70, -70, 40) → (2, 142, 40)
    public static Double[] blueTargetLocation = new Double[]{2.0, 142.0, 40.0};

    // Calibration constants for launcher trajectory calculations {velocity, gravity}
    public static Double[] launcherCalcConstants = new Double[]{280.00, 386.09};

    // ===== RED SIDE CONFIGURATIONS =====
    // Fine-tuned constants specifically for red alliance side

    // Red side azimuth fine adjustment offset (in degrees)
    public static Double RedAzimuthFineAdjustment = -6.0;

    // Red side elevation offset
    public static Double RedElevationOffset = 0.0;

    // Red side launcher calibration constants {velocity, gravity}
    public static Double[] RedLauncherCalcConstants = new Double[]{280.00, 386.09};

    // Red side launcher RPM
    public static int RedLauncherRPM = 2300;

    // Red side RPM zones
    public static double[][] RedLauncherRPMZones = new double[][]{
            {0.0, 2000},    // 0-120 inches: close range, lower RPM
            {120.0, 2500},  // 120+ inches: far range, higher RPM
    };

    // Red side AprilTag heading offset (in degrees)
    public static double RedAprilTagHeadingOffset = 90.0;

    // ===== BLUE SIDE CONFIGURATIONS =====
    // Fine-tuned constants specifically for blue alliance side

    // Blue side azimuth fine adjustment offset (in degrees)
    public static Double BlueAzimuthFineAdjustment = 0.0;

    // Blue side elevation offset
    public static Double BlueElevationOffset = 0.0;

    // Blue side launcher calibration constants {velocity, gravity}
    public static Double[] BlueLauncherCalcConstants = new Double[]{280.00, 386.09};

    // Blue side launcher RPM
    public static int BlueLauncherRPM = 2300;

    // Blue side RPM zones
    public static double[][] BlueLauncherRPMZones = new double[][]{
            {0.0, 2000},    // 0-120 inches: close range, lower RPM
            {120.0, 2500},  // 120+ inches: far range, higher RPM
    };

    // Blue side AprilTag heading offset (in degrees)
    public static double BlueAprilTagHeadingOffset = 90.0;

    // ===== SHARED/DEFAULT CONFIGURATIONS =====
    // These are used as fallbacks or when side-specific values aren't needed

    // Servo position for the lifter when raised
    public static Double LifterPositionHigh = 0.8;

    // Servo position for the lifter when lowered
    public static Double LifterPositionLow = 0.0;

    // Boolean array indicating which lifter motors/servos should be reversed
    public static boolean[] LifterReverseMap = new boolean[]{true, false, false};

    // Delay in milliseconds to wait for the lifter to reach the top position
    public static int LifterWaitToTopTimerMillis = 200;

    // Offset in degrees for the turret azimuth relative to the robot's IMU zero
    public static Double AzimuthIMUOffset = -5.0;

    // Enable correction of turret angle based on robot heading (IMU)
    public static boolean EnableTurretIMUCorrection = true;

    // Multiplier for azimuth control signals (e.g. gear ratio adjustment)
    public static Double AzimuthMultiplier = 1.0;

    // Master switch to enable or disable turret functionality
    public static boolean EnableTurret = true;

    // Allow the launcher calculator to automatically control the azimuth angle
    public static boolean EnableLauncherCalcAzimuth = true;

    // Fine adjustment offset for azimuth aiming (in degrees)
    // Positive values rotate turret clockwise, negative counter-clockwise
    public static Double AzimuthFineAdjustment = 0.0;

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

    // Offset applied to handle azimuth wrap-around logic (180.0 = 1 full rotation max)
    public static Double AzimuthWrapAroundOffset = 180.0;
    // PIDF constants for controlling the azimuth servos {kP, kI, kD, kF}
    public static double[] AzimuthPIDFConstants = new double[]{4.0, 0.2, 0.0, 0.0};

    // Servo center offset to correct for asymmetric dead band in continuous rotation servos
    // Positive values shift the "stop" point higher (e.g., 0.02 means stop is at 0.52 instead of 0.50)
    // Negative values shift it lower. Tune this if servo moves faster in one direction than the other.
    public static double AzimuthServoCenterOffset = 0.0;

    // PID deadzone as a percentage of the tolerance range (0.0 to 1.0)
    // Lower values = more responsive but may cause jitter; Higher values = less jitter but less precise
    // Default is 0.1 (10% of range), set lower like 0.05 (5%) for tighter control
    public static double AzimuthPIDDeadzonePercent = 0.05;
    public static double[] CameraOffset = new double[]{-4.2, -7.5, -12.7}; // {x, y, z} offsets of the camera from robot center in inches
    // Maximum distance from an AprilTag to use it for localization (in inches)
    public static double AprilTagMaxDistance = 60.0;

    // Heading offset to apply when relocalizing from AprilTag (in degrees)
    // This corrects for differences between AprilTag coordinate system and robot coordinate system
    // The AprilTag library returns heading in field coordinates, but the zero reference may differ
    // from your robot's IMU zero or Pedro Pathing's expected heading
    //
    // TUNING:
    // 1. Place robot at a known position facing a known direction
    // 2. Look at "AprilTag Heading (rad)" in telemetry
    // 3. Compare to expected heading in your coordinate system
    // 4. Adjust this offset until robot localizes correctly
    //
    // Common values:
    //   0.0  = No offset (AprilTag heading matches Pedro heading)
    //   90.0 = AprilTag forward is robot right (rotate 90° CCW)
    //  -90.0 = AprilTag forward is robot left (rotate 90° CW)
    //  180.0 = AprilTag forward is robot backward
    public static double AprilTagHeadingOffset = 90.0;

    // Enable threaded drive control for more responsive driving
    public static boolean EnableThreadedDrive = true;

    // Enable field-centric drive (true) or robot-centric drive (false)
    // Field-centric: Forward on stick always moves robot forward relative to field
    // Robot-centric: Forward on stick moves robot in direction it's facing
    public static boolean EnableFieldCentricDrive = true;

    // Enable threaded follower.update() to offload odometry calculations from main loop
    // This is the KEY performance optimization - follower.update() is very expensive
    public static boolean EnableThreadedFollowerUpdate = true;

    // AprilTag update frequency in milliseconds (higher = less CPU usage, lower = more responsive)
    public static int AprilTagUpdateIntervalMs = 100;
	public static int LauncherRPM = 2300;
	public static CustomPIDFController LauncherPIDF = new CustomPIDFController(20, 2, 0, 3);

	// ===== LAUNCHER RPM ZONES =====
	// RPM zones based on distance to target (in inches)
	// Format: {distance threshold (inches), RPM for that zone}
	// Distances are checked in order, first matching threshold wins
	// Example: If distance is 50 inches and zones are {{0, 2000}, {70, 2800}}
	// then RPM will be 2000 if distance < 70 inches, or 2800 if distance >= 70 inches
	public static double[][] LauncherRPMZones = new double[][]{
			{0.0, 2000},    // 0-70 inches: close range, lower RPM
			{120.0, 2500},   // 110+ inches: far range, higher RPM
	};

	// Enable RPM zones (if false, uses static LauncherRPM)
	public static boolean EnableLauncherRPMZones = true;

	// ===== EMERGENCY STOP SETTINGS =====
	// Temperature at which OpMode automatically stops (in Celsius)
	// CPU overheating is caused by intensive processing (vision, calculations), not motor current
	// Motors are on separate boards (expansion hubs) and don't heat the Control Hub CPU
	public static double EmergencyStopTemp = 88.0;

	// ===== START POSITION SETTINGS =====
	// Default starting positions for manual selection when no auto pose is available
	// Format: {x, y, heading (degrees)}
	// These can be tuned via Panels during init
	// Pedro Pathing coords: x_new = y_old + 72, y_new = -x_old + 72, heading_new = heading_old - 90
	public static double[] RedCloseStartPose = new double[]{121.15, 124.85, -130.7};
	public static double[] RedFarStartPose = new double[]{72.0, 72.0, -90.0};
	public static double[] BlueCloseStartPose = new double[]{22.85, 124.85, -40.7};
	public static double[] BlueFarStartPose = new double[]{72.0, 72.0, -90.0};
}
