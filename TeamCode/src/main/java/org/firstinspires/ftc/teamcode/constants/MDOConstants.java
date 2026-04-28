package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

import com.seattlesolvers.solverslib.controller.PIDFController;

@Config
public class MDOConstants {
	public static double AprilTagMaxDistance = 60.0;
	public static int AprilTagUpdateIntervalMs = 100;
	public static Double AzimuthFineAdjustment = 0.0;
	public static Double AzimuthIMUOffset = 110.0;
	public static Double AzimuthMultiplier = 1.0;
	public static double[] AzimuthPIDFConstants = new double[]{0.007, 0.006, 0.0, 0.0};
	public static double AzimuthServoCenterOffset = 0.0;
	public static double AzimuthServoDeadBandPositive = 0.05; // Minimum power to overcome dead band when moving in positive direction
	public static double AzimuthServoDeadBandNegative = 0.05; // Minimum power to overcome dead band when moving in negative direction
	public static double AzimuthSlewRate = 0.1; // Max power change per PID loop iteration (prevents shaking)
	public static int BallIntakeTimerMs = 1200;
	public static double BallDetectionDistanceCm = 25;
	/** Motor power (0-1) applied when the intake is running IN or OUT. */
	public static double IntakePower = 1.0;
	/** Readings above this value are treated as out-of-range / sensor error (VL53L0X reports ~819 cm when no target). */
	public static double BallSensorMaxValidDistanceCm = 300.0;
	/** Consecutive out-of-range reads before the sensor is flagged as stuck and given a longer recovery delay. */
	public static int BallSensorStuckThreshold = 25;
	public static double BlueAprilTagHeadingOffset = 90.0;
	public static Double BlueAzimuthFineAdjustment = 0.0;
	public static double[] BlueCloseStartPose = new double[]{22.85, 124.85, -40.7};
	public static double[] BlueFarStartPose = new double[]{72.0, 72.0, -90.0};
	public static Double[] BlueLauncherCalcConstants = new Double[]{280.00, 386.09};
	public static int BlueLauncherRPM = 3800;
	public static double[][] BlueLauncherRPMZones = new double[][]{
			{0.0, 3300},
			{110.0, 6000},
	};
	public static double[][] BlueElevationOffsetZones = new double[][]{
			{0.0, 0},
			{110, -0.5},
	};
	public static Double[] blueTargetLocation = new Double[]{2.0, 142.0, 40.0};
	public static Pose blueCloseLaunchLocation = new Pose(65, 80, -1.8);
	public static Pose blueHumanLocation = new Pose(144 - 39, 16, 0);
	public static Pose blueGateLocation = new Pose(27, 65, -Math.PI);
	public static double[] CameraOffset = new double[]{-4.2, -7.5, -12.7};
	public static double ColorMinBrightness = 0.0;
	public static int ColorMinRawSum = 5;
	public static double ColorMinSaturation = 0.10;
	public static int[] ColorSensorPitMapping = new int[]{0, 1, 4, 5, 2, 3};
	public static Double ElevationClampMax = 0.2;
	public static Double ElevationClampMin = -1.0;
	public static Double ElevationMax = 0.3;
	public static Double ElevationMaxIN = 14.0;
	public static Double ElevationMin = -0.4;
	public static Double ElevationMinIN = 2.0;
	public static Double ElevationMultiplier = 1.0;
	public static Double ElevationOffset = 0.0;
	/** RPM correction factor for V2 elevation: degrees per RPM deviation from nominal. */
	public static double ElevationRPMCorrectionFactor = 0.0;
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
	public static PIDFController LauncherPIDF = new PIDFController(8, 0, 2, 0);
	public static boolean EnableLauncherPID = true;
	public static int LauncherReverseRPM = 800;
	public static double LauncherManualPower = 1.0;
	public static double launchPower = 1;
	public static Double LifterPositionHigh = 0.8;
	public static Double LifterPositionLow = 0.0;
	public static int[] LifterPitMapping = new int[]{0, 1, 2};
	public static boolean[] LifterReverseMap = new boolean[]{false, true, false};
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
			{0.0, 3300},
			{110.0, 4200},
	};
	public static double[][] RedElevationOffsetZones = new double[][]{
			{0.0, 0},
			{110, -0.5},
	};
	public static Double[] redTargetLocation = new Double[]{142.0, 142.0, 40.0};
	public static Pose redCloseLaunchLocation = new Pose(89, 90, Math.PI - 1.31);
	public static Pose redHumanLocation = new Pose(144, 0, Math.PI);
	public static Pose redGateLocation = new Pose(144 - 27, 65, 0);
	public static boolean useAprilTags = true;
	/**
	 * Gain for servo position voltage-sag compensation.
	 * k in: compensated_deg = raw_deg / (1 - k * |power|).
	 * Set to 0 to disable.  Tune on FTC Dashboard.
	 */
	public static double VoltageSagCompensationGain = 0.0;

	// ===== Forward Aim Mode =====
	/**
	 * When true: turret locks to forward (servo = 0), and gamepad2 dpad adjusts
	 * elevation (up/down) and launcher speed (left/right) manually.
	 * When false: normal auto-aiming/auto-elevation behaviour is unchanged.
	 */
	public static boolean EnableForwardAimMode = false;
	/** Elevation servo position step per dpad press in forward aim mode. */
	public static double ForwardAimElevationStep = 0.02;
	/** Launcher RPM step per dpad press in forward aim mode. */
	public static int ForwardAimRPMStep = 100;
	/** Starting RPM when forward aim mode is first used. */
	public static int ForwardAimInitialRPM = 6000;
	/** Fixed RPM used for launching in forward aim mode (dashboard-tunable). */
	public static int ForwardAimRPM = 6000;
	/** Starting elevation servo position when forward aim mode is first used. */
	public static double ForwardAimInitialElevation = 0.0;
	/**
	 * Maximum turret angle (degrees) reachable when the right stick is at full deflection.
	 * Stick range [-1, 1] maps linearly to [-ForwardAimAngleRange, +ForwardAimAngleRange].
	 */
	public static double ForwardAimAngleRange = 90.0;

	// ===== ElevationSubsystemV2 — RPM-based elevation =====
	/**
	 * Launcher RPM below which the elevation is driven to maximum
	 * (shoot as steeply upward as possible).
	 * This handles the "not yet up to speed" / low-power case.
	 */
	public static double ElevationV2RPMThreshold = 1000.0;

	/**
	 * Reference launch velocity (inches/s) measured at ElevationV2BaseRPM.
	 * Velocity scales linearly with RPM: v = ElevationV2BaseVelocity * (rpm / ElevationV2BaseRPM).
	 * Initialised from the existing RedLauncherCalcConstants[0] value.
	 */
	public static double ElevationV2BaseVelocity = 280.0;

	/**
	 * Launcher RPM at which ElevationV2BaseVelocity was characterised.
	 */
	public static double ElevationV2BaseRPM = 3800.0;

	/**
	 * Effective gravity constant (inches/s²) for projectile calculations.
	 * Standard 1 g ≈ 386.09 in/s².  Tune to compensate for drag.
	 */
	public static double ElevationV2Gravity = 386.09;

	/**
	 * Scales the computed physical launch angle (radians) to servo position units.
	 * Derived from the V1 linear calibration: at baseRPM, 0.0593 rad → servo -0.4 and
	 * 0.4857 rad → servo 0.3, giving scale ≈ 1.64.
	 */
	public static double ElevationV2ServoScale = 1.64;

	/**
	 * Additive servo-position offset applied after the angle→servo mapping.
	 * Derived from the V1 calibration: offset ≈ -0.497.  Tune on dashboard.
	 */
	public static double ElevationV2ServoOffset = -0.497;
}
