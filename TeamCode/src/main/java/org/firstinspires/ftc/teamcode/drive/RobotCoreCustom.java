package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.teamcode.util.DashboardTelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.lynx.LynxModule;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;

import java.util.Arrays;
import java.util.Dictionary;
import java.util.HashMap;
import java.util.Hashtable;

public class RobotCoreCustom {
	IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
			RevHubOrientationOnRobot.LogoFacingDirection.UP,
			RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
	));
	static IMU imuEX;
	Follower follower;

	public RobotCoreCustom(HardwareMap hardwareMap, Follower follower) {
		List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
		for (LynxModule module : allHubs) {
			module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
		}
		imuEX = hardwareMap.get(IMU.class, "imuEX");
		imuEX.initialize(parameters);
		this.follower = follower;

	}

	public static Double getExternalHeading() {
		YawPitchRollAngles robotOrientation = imuEX.getRobotYawPitchRollAngles();

		// Extract the yaw angle and convert to degrees
		return robotOrientation.getYaw(AngleUnit.DEGREES);
	}

	public static Double[] localizerLauncherCalc(Follower follower, Double[] target) {
		return localizerLauncherCalc(follower, target, true); // Default to red side
	}

	public static Double[] localizerLauncherCalc(Follower follower, Double[] target, boolean isRedSide) {
		if (follower == null || target == null || target.length < 3) {
			return null;
		}
		// Get pose once and pass to thread-safe overload
		com.pedropathing.geometry.Pose currentPose = follower.getPose();
		if (currentPose == null) {
			return null;
		}
		return localizerLauncherCalc(currentPose, target, isRedSide);
	}

	/**
	 * Thread-safe overload that accepts a pre-fetched Pose
	 * Use this when calling from threaded code to avoid race conditions
	 */
	public static Double[] localizerLauncherCalc(com.pedropathing.geometry.Pose pose, Double[] target, boolean isRedSide) {
		if (pose == null || target == null || target.length < 3) {
			return null;
		}
		Double[] poseArr = {pose.getX(), pose.getY(), pose.getHeading()};
		// Use alliance-specific launcher constants
		Double[] launcherConstants = isRedSide ? MDOConstants.RedLauncherCalcConstants : MDOConstants.BlueLauncherCalcConstants;
		// launch position (x, y, z)
		// target position (x, y, z)
		// launch velocity (in/s)
		// gravity (in/s^2)
		return LocalizationAutoAim.calculateLaunchAngle(
				new Double[]{poseArr[0], poseArr[1], 10.0}, // launch position (x, y, z)
				new Double[]{target[0], target[1], target[2]}, // target position (x, y, z)
				launcherConstants[0], // launch velocity (in/s)
				launcherConstants[1] // gravity (in/s^2)
		);
	}

	/**
	 * Calculate the appropriate launcher RPM based on distance to target
	 * @param follower Robot follower for position
	 * @param target Target position [x, y, z]
	 * @return RPM value based on distance zones, or static LauncherRPM if zones disabled
	 */
	public static int calculateLauncherRPM(Follower follower, Double[] target) {
		return calculateLauncherRPM(follower, target, true); // Default to red side
	}

	/**
	 * Calculate the appropriate launcher RPM based on distance to target with alliance-specific tuning
	 * @param follower Robot follower for position
	 * @param target Target position [x, y, z]
	 * @param isRedSide true for red alliance, false for blue alliance
	 * @return RPM value based on distance zones, or static LauncherRPM if zones disabled
	 */
	public static int calculateLauncherRPM(Follower follower, Double[] target, boolean isRedSide) {
		// Get alliance-specific values
		int staticRPM = isRedSide ? MDOConstants.RedLauncherRPM : MDOConstants.BlueLauncherRPM;

		if (follower == null) {
			return staticRPM;
		}
		// Get pose once and pass to thread-safe overload
		com.pedropathing.geometry.Pose currentPose = follower.getPose();
		return calculateLauncherRPM(currentPose, target, isRedSide);
	}

	/**
	 * Thread-safe overload that accepts a pre-fetched Pose
	 * Use this when calling from threaded code to avoid race conditions
	 */
	public static int calculateLauncherRPM(com.pedropathing.geometry.Pose pose, Double[] target, boolean isRedSide) {
		// Get alliance-specific values
		int staticRPM = isRedSide ? MDOConstants.RedLauncherRPM : MDOConstants.BlueLauncherRPM;
		double[][] rpmZones = isRedSide ? MDOConstants.RedLauncherRPMZones : MDOConstants.BlueLauncherRPMZones;

		// If zones are disabled, return static RPM
		if (!MDOConstants.EnableLauncherRPMZones) {
			return staticRPM;
		}

		// Calculate distance to target
		if (pose == null || target == null || target.length < 3) {
			return staticRPM; // Fallback to default
		}

		Double[] robotPos = {pose.getX(), pose.getY(), 10.0};
		double distance = LocalizationAutoAim.getDistance(robotPos, target);

		// Find appropriate RPM zone based on distance
		// Zones are ordered by distance threshold, find the first one that matches
		int rpm = staticRPM; // Default fallback

		for (int i = rpmZones.length - 1; i >= 0; i--) {
			if (distance >= rpmZones[i][0]) {
				rpm = (int) rpmZones[i][1];
				break;
			}
		}

		return rpm;
	}

	/**
	 * Calculate the elevation offset based on distance to target with alliance-specific tuning
	 * @param follower Robot follower for position
	 * @param target Target position [x, y, z]
	 * @param isRedSide true for red alliance, false for blue alliance
	 * @return Elevation offset value based on distance zones
	 */
	public static double calculateElevationOffset(Follower follower, Double[] target, boolean isRedSide) {
		if (follower == null) {
			return 0.0;
		}
		// Get pose once and pass to thread-safe overload
		com.pedropathing.geometry.Pose currentPose = follower.getPose();
		return calculateElevationOffset(currentPose, target, isRedSide);
	}

	/**
	 * Thread-safe overload that accepts a pre-fetched Pose
	 * Use this when calling from threaded code to avoid race conditions
	 */
	public static double calculateElevationOffset(com.pedropathing.geometry.Pose pose, Double[] target, boolean isRedSide) {
		// Get alliance-specific zones
		double[][] elevationOffsetZones = isRedSide ? MDOConstants.RedElevationOffsetZones : MDOConstants.BlueElevationOffsetZones;

		// Calculate distance to target
		if (pose == null || target == null || target.length < 3) {
			return 0.0; // Fallback to no offset
		}

		Double[] robotPos = {pose.getX(), pose.getY(), 10.0};
		double distance = LocalizationAutoAim.getDistance(robotPos, target);

		// Find appropriate elevation offset zone based on distance
		// Zones are ordered by distance threshold, find the first one that matches
		double offset = 0.0; // Default fallback

		for (int i = elevationOffsetZones.length - 1; i >= 0; i--) {
			if (distance >= elevationOffsetZones[i][0]) {
				offset = elevationOffsetZones[i][1];
				break;
			}
		}

		return offset;
	}

	public void drawCurrent(Follower follower) {
		try {
			Drawing.drawRobot(follower.getPose());
			Drawing.sendPacket();
		} catch (Exception e) {
			throw new RuntimeException("Drawing failed " + e);
		}
	}

	/**
	 * Thread-safe version of drawCurrent that takes a pre-fetched Pose
	 * Use this when calling from a background thread to avoid race conditions
	 * @param pose The pre-fetched pose to draw
	 */
	public void drawCurrentWithPose(com.pedropathing.geometry.Pose pose) {
		try {
			if (pose != null) {
				Drawing.drawRobot(pose);
				Drawing.sendPacket();
			}
		} catch (Exception e) {
			// Log but don't crash - drawing is non-critical
			System.err.println("Drawing failed: " + e.getMessage());
		}
	}

	public void drawCurrentAndHistory(Follower follower) {
		PoseHistory poseHistory = follower.getPoseHistory();
		Drawing.drawPoseHistory(poseHistory);
		drawCurrent(follower);
	}
	// Get RPM over a fixed time window

	public static class CustomMotorController {
		private final String[] motorGroup;
		HashMap<String, CustomMotor> motors;
		boolean[] reverseMap;
		boolean[] encoderReverseMap;
		boolean[] encoderEnableMap; // Which motors have encoders enabled
		private volatile int targetRPM = 0;
		private volatile double targetPower = 0.0;
		private boolean isRPMMode = false;

		// Single shared PIDF controller for group-level RPM control
		private CustomPIDFController groupPidfController;
		private double lastAppliedGroupPower = 123456.0; // Impossible initial value for caching

		// Lock object for thread safety
		private final Object lock = new Object();

		/**
		 * Constructor for motor group controller (all motors same encoder setting)
		 * @param hardwareMap HardwareMap to get motors
		 * @param motorGroup Array of motor names in the group
		 * @param reverseMap Array of booleans indicating if each motor is reversed
		 * @param hasEncoder Whether motors have encoders (applies to all motors in group)
		 * @param ticksPerRev Ticks per revolution for the motors
		 * @param pidfController PID controller for RPM mode (shared across all motors)
		 */
		public CustomMotorController(HardwareMap hardwareMap, String[] motorGroup, boolean[] reverseMap, boolean hasEncoder, double ticksPerRev, CustomPIDFController pidfController) {
			this(hardwareMap, motorGroup, reverseMap, null, null, ticksPerRev, pidfController);
			// Create encoderEnableMap with all same value
			this.encoderEnableMap = new boolean[motorGroup.length];
			for (int i = 0; i < motorGroup.length; i++) {
				this.encoderEnableMap[i] = hasEncoder;
			}
			// Re-initialize motors with correct encoder settings
			for (int i = 0; i < motorGroup.length; i++) {
				String motorName = motorGroup[i];
				boolean reverseEnc = (encoderReverseMap != null) ? encoderReverseMap[i] : false;
				try {
					CustomMotor motor = new CustomMotor(hardwareMap, motorName, hasEncoder, ticksPerRev, null, reverseEnc);
					this.motors.put(motorName, motor);
				} catch (Exception e) {
					throw new IllegalArgumentException("Failed to initialize motor: " + motorName + " - " + e.getMessage());
				}
			}
		}

		/**
		 * Constructor for motor group controller with encoder reverse map
		 * @param hardwareMap HardwareMap to get motors
		 * @param motorGroup Array of motor names in the group
		 * @param reverseMap Array of booleans indicating if each motor is reversed
		 * @param encoderReverseMap Array of booleans indicating if each motor's encoder is reversed (null = no reversal)
		 * @param hasEncoder Whether motors have encoders (applies to all motors in group)
		 * @param ticksPerRev Ticks per revolution for the motors
		 * @param pidfController PID controller for RPM mode (shared across all motors)
		 */
		public CustomMotorController(HardwareMap hardwareMap, String[] motorGroup, boolean[] reverseMap, boolean[] encoderReverseMap, boolean hasEncoder, double ticksPerRev, CustomPIDFController pidfController) {
			this(hardwareMap, motorGroup, reverseMap, encoderReverseMap, null, ticksPerRev, pidfController);
			// Create encoderEnableMap with all same value
			this.encoderEnableMap = new boolean[motorGroup.length];
			for (int i = 0; i < motorGroup.length; i++) {
				this.encoderEnableMap[i] = hasEncoder;
			}
			// Re-initialize motors with correct encoder settings
			for (int i = 0; i < motorGroup.length; i++) {
				String motorName = motorGroup[i];
				boolean reverseEnc = (encoderReverseMap != null) ? encoderReverseMap[i] : false;
				try {
					CustomMotor motor = new CustomMotor(hardwareMap, motorName, hasEncoder, ticksPerRev, null, reverseEnc);
					this.motors.put(motorName, motor);
				} catch (Exception e) {
					throw new IllegalArgumentException("Failed to initialize motor: " + motorName + " - " + e.getMessage());
				}
			}
		}

		/**
		 * Constructor for motor group controller with per-motor encoder enable map
		 * @param hardwareMap HardwareMap to get motors
		 * @param motorGroup Array of motor names in the group
		 * @param reverseMap Array of booleans indicating if each motor is reversed
		 * @param encoderReverseMap Array of booleans indicating if each motor's encoder is reversed (null = no reversal)
		 * @param encoderEnableMap Array of booleans indicating which motors have encoders (null = all disabled)
		 * @param ticksPerRev Ticks per revolution for the motors
		 * @param pidfController PID controller for RPM mode (shared across all motors)
		 */
		public CustomMotorController(HardwareMap hardwareMap, String[] motorGroup, boolean[] reverseMap, boolean[] encoderReverseMap, boolean[] encoderEnableMap, double ticksPerRev, CustomPIDFController pidfController) {
			this.motorGroup = motorGroup;
			this.reverseMap = reverseMap;
			this.encoderReverseMap = encoderReverseMap;
			this.encoderEnableMap = encoderEnableMap != null ? encoderEnableMap : new boolean[motorGroup.length];
			this.motors = new HashMap<>();
			this.groupPidfController = pidfController;

			if (motorGroup.length != reverseMap.length) {
				throw new IllegalArgumentException("Motor group and reverse map must have the same length.");
			}
			if (encoderReverseMap != null && motorGroup.length != encoderReverseMap.length) {
				throw new IllegalArgumentException("Motor group and encoder reverse map must have the same length.");
			}
			if (encoderEnableMap != null && motorGroup.length != encoderEnableMap.length) {
				throw new IllegalArgumentException("Motor group and encoder enable map must have the same length.");
			}

			for (int i = 0; i < motorGroup.length; i++) {
				String motorName = motorGroup[i];
				boolean reverseEnc = (encoderReverseMap != null) ? encoderReverseMap[i] : false;
				boolean hasEnc = this.encoderEnableMap[i];
				try {
					// Motors don't need individual PIDF controllers - group controller handles PID
					CustomMotor motor = new CustomMotor(hardwareMap, motorName, hasEnc, ticksPerRev, null, reverseEnc);
					this.motors.put(motorName, motor);
				} catch (Exception e) {
					throw new IllegalArgumentException("Failed to initialize motor: " + motorName + " - " + e.getMessage());
				}
			}
		}

		/**
		 * Set RPM for all motors in the group
		 * @param rpm Target RPM
		 */
		public void setRPM(int rpm) {
			synchronized (lock) {
				this.targetRPM = rpm;
				this.isRPMMode = true;
			}
		}

		/**
		 * Set power for all motors in the group
		 * @param power Target power [-1, 1]
		 */
		public void setPower(double power) {
			synchronized (lock) {
				this.targetPower = power;
				this.isRPMMode = false;
				for (int i = 0; i < motorGroup.length; i++) {
					String motorName = motorGroup[i];
					CustomMotor motor = motors.get(motorName);
					if (motor != null) {
						double finalPower = reverseMap[i] ? -power : power;
						motor.setPower(finalPower);
					}
				}
			}
		}

		/**
		 * Update RPM PID for all motors in the group - call this in a loop.
		 * Uses a single PIDF calculation based on average RPM, then applies the SAME power to all motors.
		 */
		public void updateRPMPID() {
			synchronized (lock) {
				if (isRPMMode && groupPidfController != null) {
					// Get average RPM from all motors (with encoder reversal already applied)
					double currentRPM = getAverageRPMInternal();

					// Single PIDF calculation for the group
					double power = groupPidfController.calculate(targetRPM, currentRPM, 0, 50);
					power = Math.max(-1.0, Math.min(1.0, power)); // Clamp power to [-1, 1]

					// Apply the SAME power to all motors (with direction reversal for motor orientation)
					if (Math.abs(power - lastAppliedGroupPower) > 0.005 || (power == 0 && lastAppliedGroupPower != 0) || (power != 0 && lastAppliedGroupPower == 0)) {
						for (int i = 0; i < motorGroup.length; i++) {
							String motorName = motorGroup[i];
							CustomMotor motor = motors.get(motorName);
							if (motor != null) {
								double finalPower = reverseMap[i] ? -power : power;
								motor.setRawPower(finalPower);
							}
						}
						lastAppliedGroupPower = power;
					}
				}
			}
		}

		/**
		 * Internal method to get average RPM without locking (caller must hold lock).
		 * Only counts motors that have encoders enabled via encoderEnableMap.
		 * Returns signed RPM value - important for PID direction control.
		 */
		private double getAverageRPMInternal() {
			double sum = 0.0;
			int count = 0;
			for (CustomMotor motor : motors.values()) {
				if (motor.hasEncoder) {
					double rpm = motor.updateAndGetRPM(); // Force fresh read for PID
					sum += rpm; // Keep sign for PID direction control
					count++;
				}
			}
			return count > 0 ? sum / count : 0.0;
		}

		/**
		 * Get average RPM of all motors in the group.
		 * Only counts motors that have encoders enabled via encoderEnableMap.
		 * @return Average RPM of enabled encoders
		 */
		public double getAverageRPM() {
			synchronized (lock) {
				double sum = 0.0;
				int count = 0;
				for (CustomMotor motor : motors.values()) {
					if (motor.hasEncoder) {
						double rpm = motor.getRPM();
						sum += Math.abs(rpm); // Use absolute value to handle reversed encoders
						count++;
					}
				}
				return count > 0 ? sum / count : 0.0;
			}
		}

		/**
		 * Get RPM of a specific motor by name
		 * @param motorName Name of the motor
		 * @return RPM of the motor
		 */
		public double getRPM(String motorName) {
			synchronized (lock) {
				CustomMotor motor = motors.get(motorName);
				if (motor != null) {
					return motor.getRPM();
				}
				throw new IllegalArgumentException("Motor not found: " + motorName);
			}
		}

		/**
		 * Set PIDF controller for group-level RPM control
		 * @param pidfController New PIDF controller (shared across all motors)
		 */
		public void setPIDFController(CustomPIDFController pidfController) {
			synchronized (lock) {
				this.groupPidfController = pidfController;
			}
		}

		/**
		 * Get the target RPM
		 * @return Target RPM
		 */
		public int getTargetRPM() {
			return targetRPM;
		}

		/**
		 * Get the target power
		 * @return Target power
		 */
		public double getTargetPower() {
			return targetPower;
		}

		/**
		 * Get the last applied PID output power
		 * @return Last applied group power from PID controller (-1 to 1)
		 */
		public double getPIDOutput() {
			synchronized (lock) {
				// Return 0 if never set (initial impossible value)
				return lastAppliedGroupPower == 123456.0 ? 0.0 : lastAppliedGroupPower;
			}
		}

		/**
		 * Check if in RPM mode
		 * @return true if in RPM mode
		 */
		public boolean isRPMMode() {
			return isRPMMode;
		}

		/**
		 * Get debug string with raw velocity values from all motors
		 * @return Debug string with raw ticks/sec, calculated RPM, and ticksPerRev for each motor
		 */
		public String getDebugString() {
			StringBuilder sb = new StringBuilder();
			for (String motorName : motorGroup) {
				CustomMotor motor = motors.get(motorName);
				if (motor != null && motor.hasEncoder) {
					double rawVel = motor.getRawVelocity();
					double rpm = motor.getRPM();
					double ticksPerRev = motor.getTicksPerRev();
					// Show expected RPM calculation: (rawVel / ticksPerRev) * 60
					double expectedRPM = (ticksPerRev > 0) ? (rawVel / ticksPerRev) * 60.0 : 0;
					sb.append(String.format("%s: vel=%.0f rpm=%.1f exp=%.1f tpr=%.0f | ",
							motorName, rawVel, rpm, expectedRPM, ticksPerRev));
				}
			}
			return sb.toString();
		}
	}

	public static class CustomMotor {
		private final com.qualcomm.robotcore.hardware.DcMotorEx motor; // Changed to DcMotorEx
		private final double TICKS_PER_REV;
		private final boolean hasEncoder;
		private boolean reverseEncoder = false;
		private boolean isRPMMode = false;
		private int targetRPM = 0;
		private double lastAppliedPower = 123456.0; // Impossible initial value

		// Thread-safe caching for RPM using lock for atomic read/write
		private volatile double cachedRPM = 0.0;
		private volatile long lastRPMUpdateTime = 0;
		private static final long RPM_CACHE_TTL_MS = 15; // Cache valid for 15ms (reduced for tighter PID)
		private final Object rpmCacheLock = new Object(); // Lock for thread-safe cache access

		// PID coefficients
		private CustomPIDFController pidfController = new CustomPIDFController(0.1, 0.01, 0.005, 0.0);

		public CustomMotor(HardwareMap hardwareMap, String motorName, Boolean hasEncoder, double ticksPerRev, CustomPIDFController pidfController) {
			this(hardwareMap, motorName, hasEncoder, ticksPerRev, pidfController, false);
		}

		public CustomMotor(HardwareMap hardwareMap, String motorName, Boolean hasEncoder, double ticksPerRev, CustomPIDFController pidfController, boolean reverseEncoder) {
			this.TICKS_PER_REV = ticksPerRev;
			this.pidfController = pidfController;
			this.reverseEncoder = reverseEncoder;
			try {
				// Get as DcMotorEx directly
				this.motor = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, motorName);
			} catch (Exception e) {
				throw new IllegalArgumentException("Motor with name " + motorName + " not found or is not a DcMotorEx.");
			}
			if (hasEncoder) {
				this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				this.hasEncoder = true;
			} else {
				this.hasEncoder = false;
			}
		}

		/**
		 * Set the encoder reverse flag
		 * @param reverse true to reverse encoder readings
		 */
		public void setReverseEncoder(boolean reverse) {
			this.reverseEncoder = reverse;
		}

		/**
		 * Get RPM using native Expansion Hub velocity calculation.
		 * Reads from hardware if cache is expired, otherwise returns cached value.
		 * Thread-safe: uses lock to prevent race conditions on cache access.
		 * Uses ticks per second and converts to RPM using the configured TICKS_PER_REV.
		 * @return RPM value (primitive double)
		 */
		public double getRPM() {
			if (!hasEncoder) {
				throw new IllegalStateException("Encoder disabled for this motor: " + motor.getDeviceName());
			}

			synchronized (rpmCacheLock) {
				long currentTime = System.currentTimeMillis();
				// If cache is fresh, return it (avoids hardware read)
				if (currentTime - lastRPMUpdateTime < RPM_CACHE_TTL_MS) {
					return cachedRPM;
				}

				// Read velocity in ticks per second from hardware
				double velocityTPS = motor.getVelocity(); // Ticks Per Second
				// Convert ticks/second to RPM: (ticks/sec) / (ticks/rev) * 60 = RPM
				double rpm = 0.0;
				if (TICKS_PER_REV > 0) {
					rpm = (velocityTPS / TICKS_PER_REV) * 60.0;
				}
				if (reverseEncoder) rpm = -rpm;

				cachedRPM = rpm;
				lastRPMUpdateTime = currentTime;

				return rpm;
			}
		}

		/**
		 * Force a fresh read of RPM from hardware.
		 * Thread-safe: uses lock to update cache atomically.
		 * Uses ticks per second and converts to RPM using the configured TICKS_PER_REV.
		 * @return RPM value (primitive double)
		 */
		public double updateAndGetRPM() {
			if (!hasEncoder) return 0.0;

			synchronized (rpmCacheLock) {
				// Read velocity in ticks per second from hardware
				double velocityTPS = motor.getVelocity(); // Ticks Per Second
				// Convert ticks/second to RPM: (ticks/sec) / (ticks/rev) * 60 = RPM
				double rpm = 0.0;
				if (TICKS_PER_REV > 0) {
					rpm = (velocityTPS / TICKS_PER_REV) * 60.0;
				}
				if (reverseEncoder) rpm = -rpm;
				cachedRPM = rpm;
				lastRPMUpdateTime = System.currentTimeMillis();
				return rpm;
			}
		}

		public void setRPM(int rpm) {
			if (!hasEncoder) {
				throw new IllegalStateException("Encoder disabled for this motor: " + motor.getDeviceName());
			}
			this.targetRPM = rpm;
			this.isRPMMode = true;
		}

		public void updateRPMPID() {
			if (isRPMMode) {
				// Force update for PID loop
				double currentRPM = updateAndGetRPM();
				double power = pidfController.calculate(targetRPM, currentRPM, 0, 50);
				power = Math.max(-1.0, Math.min(1.0, power)); // Clamp power to [-1, 1]

				// Write cache checking
				if (Math.abs(power - lastAppliedPower) > 0.005 || (power == 0 && lastAppliedPower != 0) || (power != 0 && lastAppliedPower == 0)) {
					motor.setPower(power);
					lastAppliedPower = power;
				}
			}

		}

		public void setPower(double power) {
			// Write cache checking
			if (Math.abs(power - lastAppliedPower) > 0.005 || (power == 0 && lastAppliedPower != 0) || (power != 0 && lastAppliedPower == 0)) {
				motor.setPower(power);
				lastAppliedPower = power;
			}
			this.isRPMMode = false;
		}

		/**
		 * Set power directly without affecting RPM mode state.
		 * Used by CustomMotorController for group-level PID control.
		 * @param power Power to set [-1, 1]
		 */
		public void setRawPower(double power) {
			motor.setPower(power);
			lastAppliedPower = power;
		}

		public void setPIDFController(CustomPIDFController pidfController) {
			this.pidfController = pidfController;
		}

		/**
		 * Get raw velocity in ticks per second for debugging
		 * @return Raw velocity from encoder (ticks/second)
		 */
		public double getRawVelocity() {
			return motor.getVelocity();
		}

		/**
		 * Get the configured ticks per revolution
		 * @return TICKS_PER_REV value
		 */
		public double getTicksPerRev() {
			return TICKS_PER_REV;
		}
	}
/**
	 * Thread-safe telemetry wrapper that buffers addData calls from any thread
	 * and flushes them in insertion order when update() is called.
	 *
	 * Usage: call addData() from any thread at any time. Call update() from the
	 * main loop thread to push all buffered entries to the driver station / dashboard.
	 */
	public static class CustomTelemetry {
		private final Telemetry combined;
		private final java.util.concurrent.ConcurrentLinkedQueue<String[]> buffer =
				new java.util.concurrent.ConcurrentLinkedQueue<>();

		public CustomTelemetry(Telemetry telemetry, DashboardTelemetryManager ignored) {
			this.combined = new com.acmerobotics.dashboard.telemetry.MultipleTelemetry(
					telemetry, com.acmerobotics.dashboard.FtcDashboard.getInstance().getTelemetry());
		}

		/**
		 * Thread-safe: buffers a telemetry entry. Can be called from any thread.
		 * Entries are displayed in the order they were added when update() is called.
		 */
		public void addData(String caption, Object value) {
			buffer.add(new String[]{caption, String.valueOf(value)});
		}

		/**
		 * Drains all buffered entries in insertion order to the underlying telemetry,
		 * then pushes the update. Should be called from the main loop thread.
		 */
		public void update() {
			String[] entry;
			while ((entry = buffer.poll()) != null) {
				combined.addData(entry[0], entry[1]);
			}
			combined.update();
		}

	}
	public static class CustomSorterController {
		public enum CustomColor {
			GREEN,
			PURPLE,
			NULL
		}
		int[] lifterState = new int[3];
		ElapsedTime[] lifterTimers = new ElapsedTime[3];
		private final Servo[] lifter = new Servo[3];
		private final ColorSensor[] colorSensor = new ColorSensor[6];
		private CustomRGBController RGBPrism;
		// Cache for colors and ball count
		private volatile CustomColor[] cachedColors = {CustomColor.NULL, CustomColor.NULL, CustomColor.NULL};
		private volatile int cachedBallCount = 0;
		// Lock lifters during launch to prevent balls from moving between pits
		private volatile boolean liftersLocked = false;

		// ===== DEBUG TELEMETRY DATA (updated in background thread) =====
		// Cached RGB values for each sensor [sensor index][R, G, B]
		private volatile int[][] cachedRGB = new int[6][3];
		// Cached HSV values for each sensor [sensor index][H, S, V]
		private volatile float[][] cachedHSV = new float[6][3];

		public CustomSorterController(HardwareMap hardwareMap) {
			lifter[0] = hardwareMap.get(Servo.class, "lifter0");
			lifter[1] = hardwareMap.get(Servo.class, "lifter1");
			lifter[2] = hardwareMap.get(Servo.class, "lifter2");
			colorSensor[0] = hardwareMap.get(ColorSensor.class, "colorSensor0");
			colorSensor[1] = hardwareMap.get(ColorSensor.class, "colorSensor0-1");
			colorSensor[2] = hardwareMap.get(ColorSensor.class, "colorSensor1");
			colorSensor[3] = hardwareMap.get(ColorSensor.class, "colorSensor1-1");
			colorSensor[4] = hardwareMap.get(ColorSensor.class, "colorSensor2");
			colorSensor[5] = hardwareMap.get(ColorSensor.class, "colorSensor2-1");
			RGBPrism = new CustomRGBController(hardwareMap, 6);
			lifterTimers[0] = new ElapsedTime();
			lifterTimers[1] = new ElapsedTime();
			lifterTimers[2] = new ElapsedTime();

			// Initialize lifters to low position
			int[] lifterMapping = MDOConstants.LifterPitMapping;
			boolean[] reverseMap = MDOConstants.LifterReverseMap;
			for (int i = 0; i < 3; i++) {
				int lifterIndex = lifterMapping[i];
				lifter[lifterIndex].setPosition(reverseMap[lifterIndex] ? 1 - MDOConstants.LifterPositionLow : MDOConstants.LifterPositionLow);
			}
		}

		/**
		 * Updates the cached color values from sensors.
		 * This involves I2C reads and should be called from a separate thread.
		 */
		public void updateSensors() {
			int ballCount = 0;
			CustomColor[] newColors = new CustomColor[3];

			// Update debug data for all 6 sensors
			int[][] newRGB = new int[6][3];
			float[][] newHSV = new float[6][3];

			for (int s = 0; s < 6; s++) {
				int argb = colorSensor[s].argb();
				int red = (argb >> 16) & 0xFF;
				int green = (argb >> 8) & 0xFF;
				int blue = argb & 0xFF;
				newRGB[s] = new int[]{red, green, blue};
				newHSV[s] = rgbToHsv(red, green, blue);
			}
			cachedRGB = newRGB;
			cachedHSV = newHSV;

			for (int i = 0; i < 3; i++) {
				int sensorIndex0, sensorIndex1;
				int[] mapping = MDOConstants.ColorSensorPitMapping;

				switch (i) {
					case 0:
						sensorIndex0 = mapping[0];
						sensorIndex1 = mapping[1];
						break;
					case 1:
						sensorIndex0 = mapping[2];
						sensorIndex1 = mapping[3];
						break;
					case 2:
						sensorIndex0 = mapping[4];
						sensorIndex1 = mapping[5];
						break;
					default:
						// Should not happen for 0-2
						continue;
				}

				// Check first sensor
				int colorARGB0 = colorSensor[sensorIndex0].argb();
				CustomColor color = calcColor(colorARGB0);

				// If first sensor fails, check second sensor
				if (color == CustomColor.NULL) {
					int colorARGB1 = colorSensor[sensorIndex1].argb();
					color = calcColor(colorARGB1);
				}

				newColors[i] = color;
				if (color != CustomColor.NULL) {
					ballCount++;
				}
			}

			cachedColors = newColors;
			cachedBallCount = ballCount;
		}

		/**
		 * Get cached color for a pit.
		 * Non-blocking, returns immediately.
		 */
		public CustomColor getCachedColor(int pitSelector) {
			if (pitSelector >= 0 && pitSelector < 3) {
				return cachedColors[pitSelector];
			}
			return CustomColor.NULL;
		}

		/**
		 * Get cached total ball count.
		 * Non-blocking.
		 */
		public int getCachedBallCount() {
			return cachedBallCount;
		}

		/**
		 * Get cached RGB values for a sensor.
		 * @param sensorIndex The sensor index (0-5)
		 * @return int array [R, G, B] (0-255 each)
		 */
		public int[] getCachedRGB(int sensorIndex) {
			if (sensorIndex >= 0 && sensorIndex < 6) {
				return cachedRGB[sensorIndex];
			}
			return new int[]{0, 0, 0};
		}

		/**
		 * Get cached HSV values for a sensor.
		 * @param sensorIndex The sensor index (0-5)
		 * @return float array [H (0-360), S (0-1), V (0-1)]
		 */
		public float[] getCachedHSV(int sensorIndex) {
			if (sensorIndex >= 0 && sensorIndex < 6) {
				return cachedHSV[sensorIndex];
			}
			return new float[]{0, 0, 0};
		}

		/**
		 * Get formatted debug string for a sensor.
		 * @param sensorIndex The sensor index (0-5)
		 * @return Formatted string with RGB, HSV values, and raw sum
		 */
		public String getDebugString(int sensorIndex) {
			int[] rgb = getCachedRGB(sensorIndex);
			float[] hsv = getCachedHSV(sensorIndex);
			int rawSum = rgb[0] + rgb[1] + rgb[2];
			return String.format("RGB(%d,%d,%d) Sum:%d H:%.0f S:%.2f V:%.2f",
					rgb[0], rgb[1], rgb[2], rawSum, hsv[0], hsv[1], hsv[2]);
		}

		/* Original blocking method kept for reference or direct use if needed */
		public CustomColor getDirectColor(int pitSelector) {
			return getColor(pitSelector);
		}

		public CustomColor getColor(int pitSelector) {

			int sensorIndex0, sensorIndex1;
			int[] mapping = MDOConstants.ColorSensorPitMapping;

			switch (pitSelector) {
				case 0:
					sensorIndex0 = mapping[0];
					sensorIndex1 = mapping[1];
					break;
				case 1:
					sensorIndex0 = mapping[2];
					sensorIndex1 = mapping[3];
					break;
				case 2:
					sensorIndex0 = mapping[4];
					sensorIndex1 = mapping[5];
					break;
				default:
					throw new IllegalArgumentException("Invalid pit selector: " + pitSelector);
			}

			int colorARGB0 = colorSensor[sensorIndex0].argb();
			CustomColor color0 = calcColor(colorARGB0);
			if (color0 != CustomColor.NULL) {
				return color0;
			} else {
				int colorARGB1 = colorSensor[sensorIndex1].argb();
				return calcColor(colorARGB1);
			}
		}

		/**
		 * Get color from a specific sensor index (0-5)
		 * @param sensorIndex The color sensor index (0-5)
		 * @return The detected color
		 */
		public CustomColor getSensorColor(int sensorIndex) {
			if (sensorIndex < 0 || sensorIndex >= 6) {
				throw new IllegalArgumentException("Invalid sensor index: " + sensorIndex);
			}
			int colorARGB = colorSensor[sensorIndex].argb();
			return calcColor(colorARGB);
		}

		/**
		 * Get raw RGB values from a specific sensor
		 * @param sensorIndex The color sensor index (0-5)
		 * @return Array of [red, green, blue] values (0-255)
		 */
		public int[] getSensorRGB(int sensorIndex) {
			if (sensorIndex < 0 || sensorIndex >= 6) {
				throw new IllegalArgumentException("Invalid sensor index: " + sensorIndex);
			}
			int argb = colorSensor[sensorIndex].argb();
			int red = (argb >> 16) & 0xFF;
			int green = (argb >> 8) & 0xFF;
			int blue = argb & 0xFF;
			return new int[]{red, green, blue};
		}

		public void launch(CustomColor color) {
			boolean isLaunched = false;
			// Priority order: slot 2 first, then 1, then 0
			int[] slotPriority = {2, 1, 0};
			if (color == CustomColor.NULL) {
				for (int i : slotPriority) {
					// Only launch from idle slots (state 0) to prevent double-launching
					if (lifterState[i] == 0 && getColor(i) != CustomColor.NULL && !isLaunched) {
						isLaunched = true;
						lifterState[i] = 1;
					}
				}
			} else {
				// First try to find the requested color in an idle slot
				for (int i : slotPriority) {
					// Only launch from idle slots (state 0) to prevent double-launching
					if (lifterState[i] == 0 && getColor(i) == color && !isLaunched) {
						isLaunched = true;
						lifterState[i] = 1;
					}
				}
				// If requested color not found in idle slot, launch any available ball from idle slot
				if (!isLaunched) {
					for (int i : slotPriority) {
						// Only launch from idle slots (state 0) to prevent double-launching
						if (lifterState[i] == 0 && getColor(i) != CustomColor.NULL && !isLaunched) {
							isLaunched = true;
							lifterState[i] = 1;
						}
					}
				}
			}
		}
		/**
		 * Launch command extended to use cached values if desired or fallback.
		 * To use cached values, ensure updateSensors() is being called periodically.
		 * @return true if a ball was found and launch was triggered, false otherwise
		 */
		public boolean launchCached(CustomColor color) {
			boolean isLaunched = false;
			// Priority order: slot 2 first, then 1, then 0
			int[] slotPriority = {2, 1, 0};
			if (color == CustomColor.NULL) {


				for (int i : slotPriority) {
					// Only launch from idle slots (state 0) to prevent double-launching
					if (lifterState[i] == 0 && getCachedColor(i) != CustomColor.NULL && !isLaunched) {
						isLaunched = true;
						lifterState[i] = 1;
					}
				}
			} else {
				// First try to find the requested color in an idle slot
				for (int i : slotPriority) {
					// Only launch from idle slots (state 0) to prevent double-launching
					if (lifterState[i] == 0 && getCachedColor(i) == color && !isLaunched) {
						isLaunched = true;
						lifterState[i] = 1;
					}
				}
				// If requested color not found in idle slot, launch any available ball from idle slot
				if (!isLaunched) {
					for (int i : slotPriority) {
						// Only launch from idle slots (state 0) to prevent double-launching
						if (lifterState[i] == 0 && getCachedColor(i) != CustomColor.NULL && !isLaunched) {
							isLaunched = true;
							lifterState[i] = 1;
						}
					}
				}
			}
			return isLaunched;
		}

		/**
		 * Launch command that ONLY launches the requested color (no fallback).
		 * Use this for sorted/ordered launching where you need a specific color.
		 * To use cached values, ensure updateSensors() is being called periodically.
		 * @param color The specific color to launch (must not be NULL)
		 * @return true if a ball of the requested color was found and launch was triggered, false otherwise
		 */
		public boolean launchCachedStrict(CustomColor color) {
			if (color == CustomColor.NULL) {
				return false; // Strict mode requires a specific color
			}

			boolean isLaunched = false;
			// Priority order: slot 2 first, then 1, then 0
			int[] slotPriority = {2, 1, 0};

			// Only launch the requested color - no fallback to other colors
			for (int i : slotPriority) {
				// Only launch from idle slots (state 0) to prevent double-launching
				if (lifterState[i] == 0 && getCachedColor(i) == color && !isLaunched) {
					isLaunched = true;
					lifterState[i] = 1;
				}
			}

			return isLaunched;
		}

		/**
		 * Lock/unlock lifters during launch to prevent balls from moving between pits.
		 * When locked, all lifters are held in the down position and lifterUpdater() is paused.
		 * @param lock true to lock lifters down, false to unlock
		 */
		public void lockLiftersForLaunch(boolean lock) {
			liftersLocked = lock;
			if (lock) {
				// Set all lifters to down position to trap balls in their current slots
				int[] lifterMapping = MDOConstants.LifterPitMapping;
				boolean[] reverseMap = MDOConstants.LifterReverseMap;
				for (int i = 0; i < 3; i++) {
					int lifterIndex = lifterMapping[i];
					// Only set to down if not currently launching (state 0)
					if (lifterState[i] == 0) {
						lifter[lifterIndex].setPosition(reverseMap[lifterIndex] ? 1 - MDOConstants.LifterPositionLow : MDOConstants.LifterPositionLow);
					}
				}
			}
		}

		/**
		 * Check if lifters are currently locked for launch.
		 * @return true if lifters are locked
		 */
		public boolean areLiftersLocked() {
			return liftersLocked;
		}

		/**
		 * Force launch from a specific slot, regardless of sensor reading.
		 * Use this as a fallback when sensors don't detect a ball.
		 * @param slot The slot index (0, 1, or 2) to force launch from
		 */
		public void forceLaunchSlot(int slot) {
			if (slot >= 0 && slot < 3) {
				lifterState[slot] = 1;
			}
		}

		/**
		 * Launch a ball from a specific pit index.
		 * Only launches if the pit is idle (not already launching) and contains a ball.
		 * If no ball detected but pit is idle, will force launch anyway for rapid fire mode.
		 * @param pitIndex The pit index (0, 1, or 2) to launch from
		 * @return true if launch was initiated, false if pit was already busy
		 */
		public boolean launchFromPit(int pitIndex) {
			if (pitIndex < 0 || pitIndex >= 3) {
				return false;
			}
			// Only launch from idle pits (state 0) to prevent interrupting a launch in progress
			if (lifterState[pitIndex] == 0) {
				lifterState[pitIndex] = 1;
				return true;
			}
			return false;
		}

		/**
		 * Get the total number of slots.
		 * @return number of slots (3)
		 */
		public int getSlotCount() {
			return 3;
		}

		public void lifterUpdater() {
			int[] lifterMapping = MDOConstants.LifterPitMapping;
			boolean[] reverseMap = MDOConstants.LifterReverseMap;
			for (int i = 0; i < 3; i++) {
				int lifterIndex = lifterMapping[i];
				if (lifterState[i] == 1) {
					lifterState[i] = 2;
					lifterTimers[i].reset();
					lifter[lifterIndex].setPosition(reverseMap[lifterIndex] ? 1 - MDOConstants.LifterPositionHigh : MDOConstants.LifterPositionHigh);
				}
				else if (lifterState[i] == 2 && lifterTimers[i].milliseconds() > MDOConstants.LifterWaitToTopTimerMillis) {
					lifterState[i] = 0;
					lifter[lifterIndex].setPosition(reverseMap[lifterIndex] ? 1 - MDOConstants.LifterPositionLow : MDOConstants.LifterPositionLow);
				}
			}
		}
		public void lightingUpdater() {
			for (int i = 0; i < 3; i++) {
				CustomColor color = getCachedColor(i);
				switch (color) {
					case GREEN:
						//RGBPrism.setThird(i, new int[]{0, 255, 0});
						//RGBPrism.setSolidColor(i * 2, i * 2 + 1, 0, 255, 0);
						break;
					case PURPLE:
						//RGBPrism.setThird(i, new int[]{128, 0, 128});
						//RGBPrism.setSolidColor(i * 2, i * 2 + 1, 128, 0, 128);
						break;
					case NULL:
						//RGBPrism.setThird(i, new int[]{0, 0, 0});
						//RGBPrism.setSolidColor(i * 2, i * 2 + 1, 0, 0, 0);
						break;
				}
			}
		}

		/**
		 * Converts RGB to HSV and classifies color based on hue, saturation, and value thresholds.
		 * This approach is more robust than simple RGB comparison and reduces false positives/negatives.
		 *
		 * HSV ranges:
		 * - Hue: 0-360 degrees (Red: 0/360, Green: ~120, Blue: ~240, Purple/Magenta: ~280-320)
		 * - Saturation: 0-1 (0 = gray, 1 = pure color)
		 * - Value: 0-1 (0 = black, 1 = bright)
		 */
		private CustomColor calcColor(int argb) {
			// Extract color components from ARGB int
			int red = (argb >> 16) & 0xFF;
			int green = (argb >> 8) & 0xFF;
			int blue = argb & 0xFF;

			// Check raw RGB sum first - if too low, no ball is present
			// This works better than HSV brightness for low-output sensors
			int rawSum = red + green + blue;
			if (rawSum < MDOConstants.ColorMinRawSum) {
				return CustomColor.NULL;
			}

			// Convert RGB to HSV for better color classification
			float[] hsv = rgbToHsv(red, green, blue);
			float hue = hsv[0];        // 0-360
			float saturation = hsv[1]; // 0-1
			float value = hsv[2];      // 0-1

			// Get thresholds from constants (or use defaults)
			float minBrightness = (float) MDOConstants.ColorMinBrightness;
			float minSaturation = (float) MDOConstants.ColorMinSaturation;
			float greenHueMin = (float) MDOConstants.GreenHueMin;
			float greenHueMax = (float) MDOConstants.GreenHueMax;
			float purpleHueMin = (float) MDOConstants.PurpleHueMin;
			float purpleHueMax = (float) MDOConstants.PurpleHueMax;

			// Check minimum brightness - if too dark, no ball is present
			// Skip this check if minBrightness is 0 (disabled for low-output sensors)
			if (minBrightness > 0 && value < minBrightness) {
				return CustomColor.NULL;
			}

			// Check minimum saturation - if too gray/white, can't determine color
			if (saturation < minSaturation) {
				return CustomColor.NULL;
			}

			// Classify based on hue ranges
			// Green: typically 80-160 degrees
			if (hue >= greenHueMin && hue <= greenHueMax) {
				return CustomColor.GREEN;
			}

			// Purple/Magenta: typically 260-320 degrees
			// Also check for blue-purple range (220-320)
			if (hue >= purpleHueMin && hue <= purpleHueMax) {
				return CustomColor.PURPLE;
			}

			// No match - return NULL
			return CustomColor.NULL;
		}

		/**
		 * Converts RGB values to HSV (Hue, Saturation, Value)
		 * @param r Red component (0-255)
		 * @param g Green component (0-255)
		 * @param b Blue component (0-255)
		 * @return float array [hue (0-360), saturation (0-1), value (0-1)]
		 */
		private float[] rgbToHsv(int r, int g, int b) {
			float rf = r / 255f;
			float gf = g / 255f;
			float bf = b / 255f;

			float max = Math.max(rf, Math.max(gf, bf));
			float min = Math.min(rf, Math.min(gf, bf));
			float delta = max - min;

			float hue = 0;
			float saturation = (max == 0) ? 0 : (delta / max);
			float value = max;

			if (delta != 0) {
				if (max == rf) {
					hue = 60 * (((gf - bf) / delta) % 6);
				} else if (max == gf) {
					hue = 60 * (((bf - rf) / delta) + 2);
				} else {
					hue = 60 * (((rf - gf) / delta) + 4);
				}
			}

			// Ensure hue is in 0-360 range
			if (hue < 0) {
				hue += 360;
			}

			return new float[]{hue, saturation, value};
		}
	}
	public static class CustomRGBController {
		GoBildaPrismDriver prism;
		public CustomRGBController(HardwareMap hardwareMap, int stripLength) {
			prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
			prism.setStripLength(stripLength);
		}
		public void setSolidColor(int startingPixel, int endingPixel, int red, int green, int blue) {
			if (startingPixel < 0 || endingPixel >= prism.getNumberOfLEDs() || startingPixel > endingPixel) {
				throw new IllegalArgumentException("Invalid pixel range: " + startingPixel + " to " + endingPixel);
			}
			PrismAnimations.Solid solid = new PrismAnimations.Solid(new org.firstinspires.ftc.teamcode.Prism.Color(red, green, blue));
			solid.setStartIndex(startingPixel);
			solid.setStopIndex(endingPixel);
			solid.setBrightness(red+green+blue / 3);
			prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solid);
		}
		public void setThird(int third, int[] rgb) {
			if (rgb.length != 3) return;
			PrismAnimations.Solid solid = new PrismAnimations.Solid(new org.firstinspires.ftc.teamcode.Prism.Color(rgb[0], rgb[1], rgb[2]));
			solid.setStartIndex(prism.getNumberOfLEDs()/third* (third -1));
			solid.setStopIndex(prism.getNumberOfLEDs()/third);
			solid.setBrightness((rgb[0]+rgb[1]+rgb[2]) / 3);
			prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solid);
			return;
		}

	}
	/**
	 * CustomAxonServoController - A smart controller for servo motors with position feedback
	 * 
	 * WHAT THIS DOES:
	 * This controller manages one or more servo motors that can rotate. It can control them in two ways:
	 * 1. Simple Mode: Tell the servo where to go (like moving a dial)
	 * 2. Smart Mode: Uses a sensor to know exactly where the servo is and automatically corrects its position
	 * 
	 * WHY IT'S USEFUL:
	 * - Controls multiple servos at once as a group
	 * - Can track rotations beyond 180 degrees (keeps counting as it spins)
	 * - Automatically adjusts if the servo drifts from its target position
	 * - Handles servos that are mounted backwards or upside-down
	 */
	public static class CustomAxonServoController {
		// CONFIGURATION: Names of all the servos we're controlling
		private final String[] servoGroup;
		
		// HARDWARE: The actual servo objects we send commands to
		HashMap<String, Servo> servo;
		
		// SENSOR: Reads the servo's actual position (only used in smart mode)
		AnalogInput aPosition;
		
		// SETTINGS: Whether we're using the smart mode with position sensor
		boolean useAnalog;
		
		// SETTINGS: Which servos are mounted backwards (true = reversed)
		boolean[] reverseMap;
		
		// SETTINGS: If the sensor reads backwards compared to servo movement, flip the correction
		boolean invertServoDirection;
		
		// PID TUNING: Numbers that control how aggressively the servo corrects its position
		double[] pidCoefficients;
		CustomPIDFController pidController;
		
		// TARGET: Where we want the servo to be (in degrees, 0-360)
		private volatile double targetPosition;

		// The current position we read from the sensor (0-360 degrees)
		private volatile double currentPosition = 0.0;

		// The maximum reading from the sensor (360 degrees = full circle)
		private static final double WRAP_THRESHOLD = 360.0;

		// The last power level we sent to the servos (-1 to 1, where 0 is stopped)
		private volatile double lastAppliedPower = 0.0;
		// Cached simple mode position to prevent duplicate writes
		private volatile double lastSimplePosition = Double.NaN;

		// SETTINGS: Whether to allow wrap-around (0/360 boundary crossing)
		// true = shortest-path wrap-around (default, good for turrets that can spin freely)
		// false = no wrap-around, uses raw linear error (good for limited-range servos like elevation)
		private boolean allowWrapAround = true;

		// THREAD SAFETY: Prevents conflicts when multiple parts of code try to use the servo at once
		private final Object lock = new Object();

		/**
		 * CONSTRUCTOR (SIMPLE VERSION) - Sets up the servo controller
		 * 
		 * HOW TO USE THIS:
		 * Create a new servo controller by providing:
		 * 
		 * @param hardwareMap - The robot's hardware map (list of all connected devices)
		 * @param servoGroup - Names of servos to control together (e.g., ["leftServo", "rightServo"])
		 * @param reverseMap - Which servos are mounted backwards (e.g., [false, true] means second servo is reversed)
		 * @param useAnalogPositionSensors - true = use smart mode with sensor, false = simple mode
		 * @param pidCoefficients - Tuning numbers for smart mode [p, i, d] - controls how aggressively it corrects
		 *                          Higher numbers = faster corrections but might overshoot
		 * @param analogPositionName - Name of the position sensor in the hardware map
		 * 
		 * EXAMPLE:
		 * new CustomAxonServoController(hardwareMap, new String[]{"arm"}, new boolean[]{false}, true, new double[]{0.01, 0, 0}, "armSensor")
		 */
		public CustomAxonServoController(HardwareMap hardwareMap, String[] servoGroup, boolean[] reverseMap, boolean useAnalogPositionSensors, double[] pidCoefficients, String analogPositionName) {
			this(hardwareMap, servoGroup, reverseMap, useAnalogPositionSensors, pidCoefficients, analogPositionName, false);
		}

		/**
		 * CONSTRUCTOR (ADVANCED VERSION) - Sets up the servo controller with direction inversion
		 * 
		 * This is the same as the simple version, but adds one more option:
		 * 
		 * @param hardwareMap - The robot's hardware map (list of all connected devices)
		 * @param servoGroup - Names of servos to control together
		 * @param reverseMap - Which servos are mounted backwards
		 * @param useAnalogPositionSensors - true = smart mode with sensor, false = simple mode
		 * @param pidCoefficients - Tuning numbers for smart mode [p, i, d]
		 * @param analogPositionName - Name of the position sensor
		 * @param invertServoDirection - If true, flips the correction direction
		 *                               (Use this when sensor reads opposite to servo movement)
		 */
		public CustomAxonServoController(HardwareMap hardwareMap, String[] servoGroup, boolean[] reverseMap, boolean useAnalogPositionSensors, double[] pidCoefficients, String analogPositionName, boolean invertServoDirection) {
			// STEP 1: Store all the settings we'll need later
			this.pidCoefficients = pidCoefficients;
			this.useAnalog = useAnalogPositionSensors;
			this.servo = new HashMap<>();
			this.servoGroup = servoGroup;
			this.reverseMap = reverseMap;
			this.invertServoDirection = invertServoDirection;
			
			// STEP 2: Create the PID controller (the "brain" that corrects position errors)
			this.pidController = new CustomPIDFController(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2], 0.0);

			// STEP 3: If using smart mode, connect to the position sensor
			if (useAnalogPositionSensors) {
				try {
					this.aPosition = hardwareMap.get(AnalogInput.class, analogPositionName);
				} catch (Exception e) {
					// If the sensor name is wrong or sensor isn't connected, show an error
					throw new IllegalArgumentException("Analog input with name " + analogPositionName + " not found in hardware map.");
				}
			}

			// STEP 4: Connect to all the servo motors in the group
			for (String servoName : servoGroup) {
				try {
					this.servo.put(servoName, hardwareMap.get(Servo.class, servoName));
				} catch (Exception e) {
					// If a servo name is wrong or servo isn't connected, show an error
					throw new IllegalArgumentException("Servo with name " + servoName + " not found in hardware map.");
				}
			}

			// STEP 5: If using smart mode, read the starting position from the sensor
			if (useAnalogPositionSensors && aPosition != null) {
				try {
					currentPosition = voltageToDegrees(aPosition.getVoltage());
					targetPosition = currentPosition;
				} catch (Exception e) {
					currentPosition = 0.0;
					targetPosition = 0.0;
				}
			}
		}


		/**
		 * COMMAND: Tell the servo(s) to move to a specific position
		 * 
		 * HOW IT WORKS:
		 * - In Simple Mode: Directly moves the servo to the position you specify
		 * - In Smart Mode: Sets a target position, and the servo will automatically correct itself to reach it
		 * 
		 * @param position - Where you want the servo to go
		 *                   Range: -1 (full left) to +1 (full right)
		 *                   Example: 0 = center, -0.5 = halfway left, 1 = all the way right
		 * 
		 * WHAT HAPPENS:
		 * - The position value (-1 to 1) gets converted to degrees (0 to 180)
		 * - If servos are reversed, the controller automatically flips their direction
		 * - In smart mode, this just sets the target - the servo will move there on its own
		 */
		public void setPosition(double position) {
			if (!useAnalog) {
				synchronized (lock) {
					// SIMPLE MODE: Only write if position changed significantly
					if (Math.abs(position - lastSimplePosition) < 0.001) {
						return;
					}
					lastSimplePosition = position;

					// Convert from our range [-1, 1] to servo's native range [0, 1]
					double mapped = (position + 1.0) / 2.0;

					// Make sure we don't go outside the valid range
					mapped = Math.max(0.0, Math.min(1.0, mapped));

					// Send the command to each servo in the group
					for (String servoName : servoGroup) {
						Servo s = this.servo.get(servoName);
						if (s != null) {
							s.setPosition(mapped);
						}
					}
				}
			} else {
				// SMART MODE: Set the target position (the PID loop will move the servo there)
				// targetPosition is volatile — no lock needed, this is a single atomic write.
				// Removing the lock here eliminates contention with the PID thread,
				// so new targets are applied instantly.

				// Convert from our range [-1, 1] to degrees [0, 360]
				double rawTargetDeg = (position + 1.0) / 2.0 * WRAP_THRESHOLD;

				if (allowWrapAround) {
					// Normalize to [0, 360)
					rawTargetDeg = normalizeDeg(rawTargetDeg);

					// FORBIDDEN ZONE CLAMPING: If target is inside the forbidden zone, clamp to nearest edge
					double forbiddenCenter = MDOConstants.AzimuthForbiddenZoneCenter;
					double forbiddenHalfWidth = MDOConstants.AzimuthForbiddenZoneWidth / 2.0;

					if (forbiddenHalfWidth > 0) {
						double distToCenter = shortestAngularDiff(rawTargetDeg, forbiddenCenter);
						if (Math.abs(distToCenter) <= forbiddenHalfWidth) {
							double forbiddenMin = normalizeDeg(forbiddenCenter - forbiddenHalfWidth - 1.0);
							double forbiddenMax = normalizeDeg(forbiddenCenter + forbiddenHalfWidth + 1.0);
							if (distToCenter <= 0) {
								rawTargetDeg = forbiddenMin;
							} else {
								rawTargetDeg = forbiddenMax;
							}
						}
					}
				}
				// When allowWrapAround is false, rawTargetDeg is used as-is (no normalization, no forbidden zone)

				// Set the target position directly (volatile write — visible to PID thread immediately)
				targetPosition = rawTargetDeg;
			}
		}
		/**
		 * COMMAND: Stop the servo and hold it at its current position
		 * 
		 * WHAT IT DOES:
		 * - Tells all servos in the group to stay exactly where they are
		 * - Like pressing "pause" - the servo won't move until you give it a new command
		 * 
		 * WHEN TO USE:
		 * - When you want to freeze the servo in place
		 * - During an emergency stop
		 * - When switching between different control modes
		 */
		 public void stopServo() {
			synchronized (lock) {
				for (String servoName : servoGroup) {
					Servo s = this.servo.get(servoName);
					if (s != null) {
						// Tell the servo to hold its current position
						s.setPosition(s.getPosition());
					}
				}
			}
		 }
		/**
		 * UPDATE LOOP: This is the "brain" that makes smart mode work
		 * 
		 * WHAT IT DOES:
		 * 1. Reads where the servo actually is (from the sensor)
		 * 2. Compares it to where you want it to be (the target)
		 * 3. Calculates how much power to apply to get there
		 * 4. Sends that power to the servos
		 * 
		 * HOW TO USE:
		 * - Call this repeatedly in a loop (many times per second)
		 * - Only needed when using smart mode (useAnalog = true)
		 * - The more often you call it, the smoother the servo moves
		 * 
		 * BOUNDARY PROTECTION:
		 * The servo will NEVER cross the forbidden zone center point.
		 * Takes the shortest path unless it would cross the forbidden center.
		 */
		public void servoPidLoop() {
			// Only run this if we're in smart mode with a position sensor
			if (useAnalog && aPosition != null) {
				try {
					// STEP 1: Read the sensor and convert voltage to degrees (0-360)
					currentPosition = voltageToDegrees(aPosition.getVoltage());

					// STEP 2: Check if we're in the forbidden zone and need to escape
					// (Only applies when wrap-around is enabled — forbidden zones are a rotational concept)
					if (allowWrapAround) {
						double forbiddenCenter = MDOConstants.AzimuthForbiddenZoneCenter;
						double forbiddenHalfWidth = MDOConstants.AzimuthForbiddenZoneWidth / 2.0;

						if (forbiddenHalfWidth > 0 && isInForbiddenZone(currentPosition, forbiddenCenter, forbiddenHalfWidth)) {
							// ESCAPE MODE: Push toward the nearest edge of the forbidden zone
							double forbiddenMin = normalizeDeg(forbiddenCenter - forbiddenHalfWidth);
							double forbiddenMax = normalizeDeg(forbiddenCenter + forbiddenHalfWidth);
							double distToMin = angularDistance(currentPosition, forbiddenMin);
							double distToMax = angularDistance(currentPosition, forbiddenMax);

							double escapePower = MDOConstants.AzimuthForbiddenZoneEscapePower;
							double power = (distToMin <= distToMax) ? escapePower : -escapePower;

							if (invertServoDirection) {
								power = -power;
							}

							lastAppliedPower = power;
							applyPowerToServos(power);
							return;
						}
					}

					// STEP 3: Compute error from current to target
					// targetPosition is volatile — reads the latest value set by setPosition() instantly
					double error = targetPosition - currentPosition;

					if (allowWrapAround) {
						// WRAP-AROUND MODE: Normalize to [-180, 180] — always gives shortest path
						while (error > 180.0) error -= 360.0;
						while (error < -180.0) error += 360.0;

						// STEP 3b: If forbidden zone enabled, check if shortest path crosses it
						double forbiddenCenter = MDOConstants.AzimuthForbiddenZoneCenter;
						double forbiddenHalfWidth = MDOConstants.AzimuthForbiddenZoneWidth / 2.0;
						if (forbiddenHalfWidth > 0) {
							boolean shortestCrosses = pathCrossesForbiddenZone(
									currentPosition, currentPosition + error, forbiddenCenter, forbiddenHalfWidth);
							if (shortestCrosses) {
								if (error > 0) {
									error -= 360.0;
								} else {
									error += 360.0;
								}
							}
						}
					}

					// STEP 4: Use the PID controller to calculate power.
					double power = pidController.calculate(
							error, 0, 0, 2, 361.0);

					// STEP 5: Clamp power to [-1, 1]
					power = Math.max(-1.0, Math.min(1.0, power));

					// STEP 5b: Asymmetric dead band compensation for continuous rotation servo.
					// The servo has a mechanical dead band around center (0.5) where
					// small power values produce no movement. This dead band is often
					// different in each direction — one direction needs more power to
					// start moving than the other, causing "stepping" / overshoot in
					// that direction.
					// Fix: if PID output is below the dead band threshold, output zero
					// instead of boosting (boosting causes overshoot → jitter).
					// If PID output is above the dead band, add the dead band offset
					// so the servo actually moves proportionally.
					double deadBandPos = MDOConstants.AzimuthServoDeadBandPositive;
					double deadBandNeg = MDOConstants.AzimuthServoDeadBandNegative;
					if (power > 0) {
						if (power < deadBandPos) {
							power = 0;
						} else {
							power = power + deadBandPos;
						}
					} else if (power < 0) {
						if (power > -deadBandNeg) {
							power = 0;
						} else {
							power = power - deadBandNeg;
						}
					}

					// STEP 6: If sensor direction is backwards, flip
					if (invertServoDirection) {
						power = -power;
					}

					// STEP 7: Slew-rate limit
					double maxDelta = MDOConstants.AzimuthSlewRate;
					double delta = power - lastAppliedPower;
					if (delta > maxDelta) {
						power = lastAppliedPower + maxDelta;
					} else if (delta < -maxDelta) {
						power = lastAppliedPower - maxDelta;
					}

					lastAppliedPower = power;

					// STEP 8: Send power to servos
					applyPowerToServos(power);
				} catch (Exception e) {
					System.err.println("Error in servo PID loop: " + e.getMessage());
				}
			}
		}

		/**
		 * Sends calculated power to all servos in the group, respecting reverse maps and center offset.
		 */
		private void applyPowerToServos(double power) {
			for (int i = 0; i < servoGroup.length; i++) {
				String servoName = servoGroup[i];
				Servo s = this.servo.get(servoName);
				if (s != null) {
					double finalPower = reverseMap[i] ? -power : power;
					// Convert power from [-1, 1] to servo range [0, 1]
					// Add center offset to correct for asymmetric dead band in continuous rotation servos
					double mapped = (finalPower + 1.0) / 2.0 + MDOConstants.AzimuthServoCenterOffset;
					mapped = Math.max(0.0, Math.min(1.0, mapped)); // Safety clamp
					s.setPosition(mapped);
				}
			}
		}

		/**
		 * Normalize an angle to [0, 360) range
		 */
		private double normalizeDeg(double deg) {
			deg = deg % 360.0;
			if (deg < 0) deg += 360.0;
			return deg;
		}

		/**
		 * Calculate the shortest signed angular difference from angle1 to angle2.
		 * Returns a value in [-180, 180].
		 * Positive means angle2 is clockwise from angle1.
		 */
		private double shortestAngularDiff(double angle1, double angle2) {
			double diff = normalizeDeg(angle1) - normalizeDeg(angle2);
			while (diff > 180.0) diff -= 360.0;
			while (diff < -180.0) diff += 360.0;
			return diff;
		}

		/**
		 * Check if moving from start to end would cross through a specific point
		 */
		private boolean pathCrossesPoint(double start, double end, double point) {
			// Normalize all to 0-360
			while (start < 0) start += 360.0;
			while (start >= 360.0) start -= 360.0;
			while (point < 0) point += 360.0;
			while (point >= 360.0) point -= 360.0;

			// Determine direction
			double distance = end - start;
			if (Math.abs(distance) < 0.001) return false; // No movement

			boolean movingCW = distance > 0;

			if (movingCW) {
				// Moving clockwise (increasing angle)
				// Check if point is between start and end
				double endNorm = start + Math.abs(distance);
				// Point crosses if it's in the swept range
				double pointCheck = point;
				if (point < start) pointCheck += 360.0;
				return pointCheck > start && pointCheck < endNorm;
			} else {
				// Moving counter-clockwise (decreasing angle)
				double endNorm = start - Math.abs(distance);
				double pointCheck = point;
				if (point > start) pointCheck -= 360.0;
				return pointCheck < start && pointCheck > endNorm;
			}
		}

		/**
		 * Check if a position is inside the forbidden zone (where the cable is)
		 * @param position The current position in degrees (0-360)
		 * @param center The center of the forbidden zone
		 * @param halfWidth Half the width of the forbidden zone
		 * @return true if position is inside the forbidden zone
		 */
		private boolean isInForbiddenZone(double position, double center, double halfWidth) {
			// Normalize position to 0-360
			while (position < 0) position += 360.0;
			while (position >= 360.0) position -= 360.0;

			// Calculate angular distance from center
			double dist = angularDistance(position, center);
			return dist <= halfWidth;
		}

		/**
		 * Calculate the shortest angular distance between two angles (0-360)
		 * @param angle1 First angle in degrees
		 * @param angle2 Second angle in degrees
		 * @return The shortest distance between the angles (0 to 180)
		 */
		private double angularDistance(double angle1, double angle2) {
			// Normalize both to 0-360
			while (angle1 < 0) angle1 += 360.0;
			while (angle1 >= 360.0) angle1 -= 360.0;
			while (angle2 < 0) angle2 += 360.0;
			while (angle2 >= 360.0) angle2 -= 360.0;

			double diff = Math.abs(angle1 - angle2);
			return Math.min(diff, 360.0 - diff);
		}

		/**
		 * Check if moving from start to end would cross through the forbidden zone
		 * @param start Starting position in degrees
		 * @param end Ending position in degrees
		 * @param forbiddenCenter Center of the forbidden zone
		 * @param halfWidth Half the width of the forbidden zone
		 * @return true if the path crosses any part of the forbidden zone
		 */
		private boolean pathCrossesForbiddenZone(double start, double end, double forbiddenCenter, double halfWidth) {
			// Check if path crosses any point within the forbidden zone
			// We check the center and both edges
			double forbiddenMin = forbiddenCenter - halfWidth;
			double forbiddenMax = forbiddenCenter + halfWidth;

			// Normalize edges
			while (forbiddenMin < 0) forbiddenMin += 360.0;
			while (forbiddenMin >= 360.0) forbiddenMin -= 360.0;
			while (forbiddenMax < 0) forbiddenMax += 360.0;
			while (forbiddenMax >= 360.0) forbiddenMax -= 360.0;

			// If path crosses center, it definitely crosses the zone
			if (pathCrossesPoint(start, end, forbiddenCenter)) return true;

			// Also check if path crosses either edge
			if (pathCrossesPoint(start, end, forbiddenMin)) return true;
			if (pathCrossesPoint(start, end, forbiddenMax)) return true;

			return false;
		}

		/**
		 * READ: Get the servo's current position (0-360 degrees)
		 *
		 * WHAT IT RETURNS:
		 * The current angle in degrees (always between 0 and 360)
		 *
		 * EXAMPLES:
		 * - 90 degrees = quarter turn
		 * - 180 degrees = half turn
		 * - 270 degrees = three-quarter turn
		 * - 360 degrees loops back to 0
		 *
		 * NOTE: Only works in smart mode (when useAnalog is true)
		 */
		public double getPosition() {
			synchronized (lock) {
				if (useAnalog) {
					return currentPosition;
				} else {
					throw new IllegalStateException("Analog position sensors are not enabled for this servo group.");
				}
			}
		}

		/**
		 * HELPER METHOD: Converts sensor voltage to degrees
		 * 
		 * HOW IT WORKS:
		 * The position sensor outputs a voltage that represents the servo's angle.
		 * This method does the math to convert that voltage into degrees (0-360).
		 *
		 * Think of it like a thermometer - it reads a voltage and converts it to a temperature,
		 * but here we're converting to an angle instead.
		 */
		private double voltageToDegrees(double voltage) {
			// Assuming voltage maps linearly from 0V=0° to maxVoltage=360°
			return (voltage / aPosition.getMaxVoltage()) * WRAP_THRESHOLD;
		}


		/**
		 * READ: Get the raw sensor reading (without accumulated rotations)
		 * 
		 * WHAT IT RETURNS:
		 * Just the current sensor reading from 0-360 degrees.
		 *
		 * NOTE: With wrap-around removed, this returns the same as getPosition()
		 */
		public double getRawPosition() {
			synchronized (lock) {
				if (useAnalog) {
					return currentPosition;
				} else {
					throw new IllegalStateException("Analog position sensors are not enabled for this servo group.");
				}
			}
		}

		/**
		 * RESET: Reset the position tracking
		 *
		 * WHAT IT DOES:
		 * - Reads the current position from the sensor
		 * - Updates internal tracking
		 *
		 * WHEN TO USE:
		 * - After the servo has been manually moved
		 * - At the beginning of a new task or routine
		 */
		public void resetPosition() {
			synchronized (lock) {
				currentPosition = voltageToDegrees(aPosition.getVoltage());
				targetPosition = currentPosition;
			}
		}
	/**
	 * DEBUG INFO: Get the power being sent to servos by the PID controller
	 * 
	 * WHAT IT RETURNS:
	 * The last power value calculated by the smart controller (-1 to 1)
	 * - Positive values = servo moving in one direction
	 * - Negative values = servo moving in opposite direction
	 * - Near 0 = servo is close to target or stopped
	 * 
	 * USE THIS FOR:
	 * - Debugging why the servo isn't moving as expected
	 * - Checking if the PID controller is working properly
	 */
	public double getPIDTargetPower() {
		return lastAppliedPower;  // volatile read is atomic
	}
	
	/**
	 * DEBUG INFO: Get the current error between target and actual position
	 * 
	 * WHAT IT RETURNS:
	 * How far off the servo is from where you want it to be (in degrees)
	 * - 0 = perfectly on target
	 * - Positive = servo needs to move forward to reach target
	 * - Negative = servo needs to move backward to reach target
	 * 
	 * EXAMPLE:
	 * If target is 90 degrees and actual is 85 degrees, error is 5 degrees
	 */
	public double getPIDError() {
		synchronized (lock) {
			return pidController.error;
		}
	}
	
	/**
	 * READ: Get where you've told the servo to go
	 * 
	 * WHAT IT RETURNS:
	 * The target position in degrees (0-360) that you set with setPosition()
	 * This is where the servo is TRYING to reach, not where it currently is
	 */
	public double getTargetPosition() {
		return targetPosition;
	}
	

	/**
	 * DEBUG INFO: Get the raw voltage from the position sensor
	 * 
	 * WHAT IT RETURNS:
	 * The actual voltage reading from the sensor (usually 0-3.3V or 0-5V)
	 * 
	 * USE THIS FOR:
	 * - Checking if the sensor is connected and working
	 * - Diagnosing sensor problems
	 * - Calibrating the sensor
	 */
	public double getAnalogVoltage() {
		if (useAnalog && aPosition != null) {
			return aPosition.getVoltage();
		}
		return 0.0;
	}
	
	/**
	 * DEBUG INFO: Get the maximum voltage the sensor can output
	 * 
	 * WHAT IT RETURNS:
	 * The highest voltage the sensor will ever read (usually 3.3V or 5V)
	 * Used for converting voltage to degrees
	 */
	public double getMaxVoltage() {
		if (useAnalog && aPosition != null) {
			return aPosition.getMaxVoltage();
		}
		return 0.0;
	}
	
	/**
	 * TUNING: Change the PID controller settings
	 * 
	 * WHAT IT DOES:
	 * Updates the numbers that control how aggressively the servo corrects its position
	 * 
	 * @param pidCoefficients - Array of 3 numbers [P, I, D]:
	 *   P (Proportional): How strongly it reacts to current error
	 *      - Higher = faster response but might overshoot
	 *      - Lower = slower, gentler movement
	 *   
	 *   I (Integral): How it corrects for persistent errors over time
	 *      - Higher = fixes stubborn errors faster but can cause oscillation
	 *      - Lower = more stable but might not fully reach target
	 *   
	 *   D (Derivative): How it dampens/slows down to prevent overshooting
	 *      - Higher = smoother approach but slower response
	 *      - Lower = faster but might oscillate
	 * 
	 * WHEN TO USE:
	 * - When the servo is moving too slowly or too fast
	 * - When the servo overshoots the target
	 * - When the servo oscillates back and forth
	 * - During testing and tuning
	 */
	public void setPIDCoefficients(double[] pidCoefficients) {
		synchronized (lock) {
			if (pidCoefficients.length < 3) {
				throw new IllegalArgumentException("PID coefficients array must have at least 3 elements: p, i, d.");
			}

			// Only update if constants changed
			if (this.pidCoefficients != null &&
				Math.abs(this.pidCoefficients[0] - pidCoefficients[0]) < 0.0001 &&
				Math.abs(this.pidCoefficients[1] - pidCoefficients[1]) < 0.0001 &&
				Math.abs(this.pidCoefficients[2] - pidCoefficients[2]) < 0.0001) {
				return;
			}

			this.pidCoefficients = pidCoefficients;
			this.pidController = new CustomPIDFController(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2], 0.0);
		}
	}

	/**
	 * TOGGLE: Enable or disable wrap-around (0°/360° boundary crossing)
	 *
	 * WHAT IT DOES:
	 * - true (default): Uses shortest-path wrap-around. The servo takes the shortest route,
	 *   even if it means crossing the 0°/360° boundary. Good for turrets that can spin freely.
	 * - false: No wrap-around. Error is computed as a raw linear difference (target - current).
	 *   The servo will NOT cross the 0°/360° boundary. Good for limited-range servos like elevation.
	 *
	 * NOTE: This only affects Smart Mode (useAnalog = true). Simple mode has no wrap-around logic.
	 *
	 * @param allow - true to enable wrap-around, false to disable it
	 */
	public void setAllowWrapAround(boolean allow) {
		synchronized (lock) {
			this.allowWrapAround = allow;
		}
	}
}
}
