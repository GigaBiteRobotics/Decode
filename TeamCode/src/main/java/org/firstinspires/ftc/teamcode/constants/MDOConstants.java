package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

import com.seattlesolvers.solverslib.controller.PIDFController;

@Config
public class MDOConstants {
	// ===== Limelight pipeline indices — must match the order shown in the Limelight web UI =====
	public static int LimelightPipelineRed = 1;  // Pipeline named "Red"
	public static int LimelightPipelineBlue = 0; // Pipeline named "Blue"
	public static int LimelightPipelineIdle = 2; // Low-FPS/low-res idle pipeline — keeps limelight warm without heat buildup
	public static double AprilTagMaxDistance = 60.0;
	public static int AprilTagUpdateIntervalMs = 100;
	// --- OLD AzimuthSubsystemV2 constants (servo-PID azimuth — replaced by Limelight TurretSubsystem) ---
	//public static Double AzimuthFineAdjustment = 0.0;
	//public static Double AzimuthIMUOffset = 110.0;
	//public static Double AzimuthMultiplier = 1.0;
	//public static double[] AzimuthPIDFConstants = new double[]{0.007, 0.006, 0.0, 0.0};
	//public static double AzimuthServoCenterOffset = 0.0;
	//public static double AzimuthServoDeadBandPositive = 0.05;
	//public static double AzimuthServoDeadBandNegative = 0.05;
	//public static double AzimuthSlewRate = 0.1;
	public static int BallIntakeTimerMs = 1200;
	public static double BallDetectionDistanceCm = 25;
	/** Motor power (0-1) applied when the intake is running IN or OUT. */
	public static double IntakePower = 1.0;
	/** Readings above this value are treated as out-of-range / sensor error (VL53L0X reports ~819 cm when no target). */
	public static double BallSensorMaxValidDistanceCm = 300.0;
	/** Consecutive out-of-range reads before the sensor is flagged as stuck and given a longer recovery delay. */
	public static int BallSensorStuckThreshold = 25;
	public static double BlueAprilTagHeadingOffset = 90.0;
	//public static Double BlueAzimuthFineAdjustment = 0.0; // OLD: AzimuthSubsystemV2
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
	//public static boolean EnableLauncherCalcAzimuth = true; // OLD: AzimuthSubsystemV2 flag
	public static boolean EnableLauncherRPMZones = true;
	// EnableThreadedDrive removed \u2014 drive is always threaded via followerUpdateThread
	public static boolean EnableThreadedFollowerUpdate = true;
	/** When false, only essential operational telemetry is sent. Disabling reduces WiFi load
	 *  and gamepad latency during competition. Toggle on FTC Dashboard when tuning. */
	public static boolean DebugTelemetry = false;
	public static boolean EnableTurret = true;
	//public static boolean EnableTurretIMUCorrection = true; // OLD: IMUSubsystem-based turret correction
	public static double GreenHueMax = 190.0;
	public static double GreenHueMin = 80.0;
	public static PIDFController LauncherPIDF = new PIDFController(0.0003, 0, 0, 0.0002);
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
	public static int[] RapidFireOrder = new int[]{1, 2, 0};
	public static double RedAprilTagHeadingOffset = 90.0;
	//public static Double RedAzimuthFineAdjustment = 0.0; // OLD: AzimuthSubsystemV2
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
	// /** Launcher RPM step per dpad press in forward aim mode (no binding in MDO currently). */
	//public static int ForwardAimRPMStep = 100;
	/** Starting RPM when forward aim mode is first used. */
	public static int ForwardAimInitialRPM = 6000;
	/** Fixed RPM used for launching in forward aim mode (dashboard-tunable). */
	public static int ForwardAimRPM = 6000;
	/** Starting elevation servo position when forward aim mode is first used. */
	public static double ForwardAimInitialElevation = 0.0;
	// ForwardAimAngleRange — stick-to-angle mapping (no right-stick turret control in MDO currently)
	//public static double ForwardAimAngleRange = 90.0;

	// ===== ElevationSubsystemV2 — unimplemented physics constants (never used by ElevationSubsystemV2.java) =====
	//public static double ElevationV2RPMThreshold = 1000.0;
	//public static double ElevationV2BaseVelocity = 280.0;
	//public static double ElevationV2BaseRPM = 3800.0;
	//public static double ElevationV2Gravity = 386.09;
	//public static double ElevationV2ServoScale = 1.64;
	//public static double ElevationV2ServoOffset = -0.497;

	// ===== TurretSubsystem / Limelight Auto-Aim =====
	/** PIDF gains for the CR-servo turret centering Limelight Tx to 0. */
	public static PIDFController TurretPIDF = new PIDFController(0.009, 0.014, 0.0003, 0.0);
	/** Maximum CR-servo power applied during turret aiming (0–1). */
	public static double TurretMaxPower = 0.6;
	/** How long (ms) a lock must be continuously lost before the launcher spins down. Prevents pulsing on brief flickers. */
	public static double LockLostDebounceMs = 150;

	// Ta → RPM power-law regression:  RPM = A * Ta ^ B
	public static double LimelightTaRPMCoeffA = 3574.97926;
	public static double LimelightTaRPMCoeffB = -0.156948;

	// Ta → elevation quadratic regression:  pitch = C*Ta² + D*Ta + E
	public static double LimelightTaElevCoeffC = -0.0475945;
	public static double LimelightTaElevCoeffD = 0.11103;
	public static double LimelightTaElevCoeffE = 0.387717;
	/** Maximum elevation servo position allowed when using Limelight Ta-based pitch. */
	public static double LimelightTaElevClampMax = 0.7;
}
