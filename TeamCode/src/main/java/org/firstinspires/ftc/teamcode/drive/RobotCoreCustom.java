package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.bylazar.telemetry.TelemetryManager;
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
		if (follower == null || target == null || target.length < 3) {
			return null;
		}
		Double[] pose = {follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading()};
		// launch position (x, y, z)
		// target position (x, y, z)
		// launch velocity (in/s)
		// gravity (in/s^2)
		return LocalizationAutoAim.calculateLaunchAngle(
				new Double[]{pose[0], pose[1], 10.0}, // launch position (x, y, z)
				new Double[]{target[0], target[1], target[2]}, // target position (x, y, z)
				MDOConstants.launcherCalcConstants[0], // launch velocity (in/s)
				MDOConstants.launcherCalcConstants[1] // gravity (in/s^2)
		);
	}

	public void drawCurrent(Follower follower) {
		try {
			Drawing.drawRobot(follower.getPose());
			Drawing.sendPacket();
		} catch (Exception e) {
			throw new RuntimeException("Drawing failed " + e);
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
		private volatile int targetRPM = 0;
		private volatile double targetPower = 0.0;
		private boolean isRPMMode = false;

		// Lock object for thread safety
		private final Object lock = new Object();

		/**
		 * Constructor for motor group controller
		 * @param hardwareMap HardwareMap to get motors
		 * @param motorGroup Array of motor names in the group
		 * @param reverseMap Array of booleans indicating if each motor is reversed
		 * @param hasEncoder Whether motors have encoders (applies to all motors in group)
		 * @param ticksPerRev Ticks per revolution for the motors
		 * @param pidfController PID controller for RPM mode
		 */
		public CustomMotorController(HardwareMap hardwareMap, String[] motorGroup, boolean[] reverseMap, boolean hasEncoder, double ticksPerRev, CustomPIDFController pidfController) {
			this.motorGroup = motorGroup;
			this.reverseMap = reverseMap;
			this.motors = new HashMap<>();

			if (motorGroup.length != reverseMap.length) {
				throw new IllegalArgumentException("Motor group and reverse map must have the same length.");
			}

			for (String motorName : motorGroup) {
				try {
					CustomMotor motor = new CustomMotor(hardwareMap, motorName, hasEncoder, ticksPerRev, pidfController);
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
				for (int i = 0; i < motorGroup.length; i++) {
					String motorName = motorGroup[i];
					CustomMotor motor = motors.get(motorName);
					if (motor != null) {
						int finalRPM = reverseMap[i] ? -rpm : rpm;
						motor.setRPM(finalRPM);
					}
				}
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
		 * Update RPM PID for all motors in the group - call this in a loop
		 */
		public void updateRPMPID() {
			synchronized (lock) {
				if (isRPMMode) {
					for (CustomMotor motor : motors.values()) {
						motor.updateRPMPID();
					}
				}
			}
		}

		/**
		 * Get average RPM of all motors in the group
		 * @return Average RPM
		 */
		public double getAverageRPM() {
			synchronized (lock) {
				double sum = 0.0;
				int count = 0;
				for (CustomMotor motor : motors.values()) {
					if (motor.hasEncoder) {
						sum += motor.getRPM();
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
		 * Set PIDF controller for all motors in the group
		 * @param pidfController New PIDF controller
		 */
		public void setPIDFController(CustomPIDFController pidfController) {
			synchronized (lock) {
				for (CustomMotor motor : motors.values()) {
					motor.setPIDFController(pidfController);
				}
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
		 * Check if in RPM mode
		 * @return true if in RPM mode
		 */
		public boolean isRPMMode() {
			return isRPMMode;
		}
	}

	public static class CustomMotor {
		private final com.qualcomm.robotcore.hardware.DcMotorEx motor; // Changed to DcMotorEx
		private final double TICKS_PER_REV;
		private final boolean hasEncoder;
		private boolean isRPMMode = false;
		private int targetRPM = 0;
		private double lastAppliedPower = 123456.0; // Impossible initial value

		// Thread-safe caching for RPM
		private volatile double cachedRPM = 0.0;
		private volatile long lastRPMUpdateTime = 0;
		private static final long RPM_CACHE_TTL_MS = 20; // Cache valid for 20ms

		// PID coefficients
		private CustomPIDFController pidfController = new CustomPIDFController(0.1, 0.01, 0.005, 0.0);

		public CustomMotor(HardwareMap hardwareMap, String motorName, Boolean hasEncoder, double ticksPerRev, CustomPIDFController pidfController) {
			this.TICKS_PER_REV = ticksPerRev;
			this.pidfController = pidfController;
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
		 * Get RPM using native Expansion Hub velocity calculation.
		 * Reads from hardware if cache is expired, otherwise returns cached value.
		 */
		public Double getRPM() {
			if (!hasEncoder) {
				throw new IllegalStateException("Encoder disabled for this motor: " + motor.getDeviceName());
			}

			long currentTime = System.currentTimeMillis();
			// If cache is fresh, return it (avoids hardware read)
			if (currentTime - lastRPMUpdateTime < RPM_CACHE_TTL_MS) {
				return cachedRPM;
			}

			// Read from hardware (this is a USB read unless in BULK AUTO/MANUAL mode)
			double velocityTPS = motor.getVelocity(); // Ticks Per Second
			double rpm = (velocityTPS / TICKS_PER_REV) * 60.0;

			cachedRPM = rpm;
			lastRPMUpdateTime = currentTime;

			return rpm;
		}

		/**
		 * Force a fresh read of RPM from hardware
		 */
		public Double updateAndGetRPM() {
			if (!hasEncoder) return 0.0;
			double velocityTPS = motor.getVelocity();
			double rpm = (velocityTPS / TICKS_PER_REV) * 60.0;
			cachedRPM = rpm;
			lastRPMUpdateTime = System.currentTimeMillis();
			return rpm;
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

		public void setPIDFController(CustomPIDFController pidfController) {
			this.pidfController = pidfController;
		}
	}
	public static class CustomTelemetry {
		private final Telemetry telemetry;
		private final TelemetryManager telemetryM;

		public CustomTelemetry(Telemetry telemetry, TelemetryManager telemetryM) {
			this.telemetry = telemetry;
			this.telemetryM = telemetryM;
		}

		public void addData(String caption, Object value) {
			telemetry.addData(caption, value);
			telemetryM.debug(caption + ": " + value.toString());
		}

		public void update() {
			telemetry.update();
			telemetryM.update();
		}

	}
	public static class CustomSorterController {
		public enum CustomColor {
			GREEN,
			PURPLE,
			NULL
		}
		int[] lifterState = new int[3];
		ElapsedTime lifterTimer;
		private final Servo[] lifter = new Servo[3];
		private final ColorSensor[] colorSensor = new ColorSensor[6];
		private CustomRGBController RGBPrism;
		// Cache for colors and ball count
		private volatile CustomColor[] cachedColors = {CustomColor.NULL, CustomColor.NULL, CustomColor.NULL};
		private volatile int cachedBallCount = 0;

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
			lifterTimer = new ElapsedTime();

			// Initialize lifters to low position
			boolean[] reverseMap = MDOConstants.LifterReverseMap;
			for (int i = 0; i < 3; i++) {
				lifter[i].setPosition(reverseMap[i] ? 1 - MDOConstants.LifterPositionLow : MDOConstants.LifterPositionLow);
			}
		}

		/**
		 * Updates the cached color values from sensors.
		 * This involves I2C reads and should be called from a separate thread.
		 */
		public void updateSensors() {
			int ballCount = 0;
			CustomColor[] newColors = new CustomColor[3];

			for (int i = 0; i < 3; i++) {
				int sensorIndex0, sensorIndex1;

				switch (i) {
					case 0:
						sensorIndex0 = 0;
						sensorIndex1 = 1;
						break;
					case 1:
						sensorIndex0 = 4;
						sensorIndex1 = 5;
						break;
					case 2:
						sensorIndex0 = 2;
						sensorIndex1 = 3;
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

		/* Original blocking method kept for reference or direct use if needed */
		public CustomColor getDirectColor(int pitSelector) {
			return getColor(pitSelector);
		}

		public CustomColor getColor(int pitSelector) {

			int sensorIndex0, sensorIndex1;

			switch (pitSelector) {
				case 0:
					sensorIndex0 = 0;
					sensorIndex1 = 1;
					break;
				case 1:
					sensorIndex0 = 4;
					sensorIndex1 = 5;
					break;
				case 2:
					sensorIndex0 = 2;
					sensorIndex1 = 3;
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
			if (color == CustomColor.NULL) {
				for (int i = 0; i < 3; i++) {
					if (getColor(i) != CustomColor.NULL && !isLaunched) {
						isLaunched = true;
						lifterState[i] = 1;
					}
				}
			} else {
				for (int i = 0; i < 3; i++) {
					if (getColor(i) == color && !isLaunched) {
						isLaunched = true;
						lifterState[i] = 1;
					}
				}
			}
		}
		/**
		 * Launch command extended to use cached values if desired or fallback.
		 * To use cached values, ensure updateSensors() is being called periodically.
		 */
		public void launchCached(CustomColor color) {
			boolean isLaunched = false;
			if (color == CustomColor.NULL) {
				for (int i = 0; i < 3; i++) {
					if (getCachedColor(i) != CustomColor.NULL && !isLaunched) {
						isLaunched = true;
						lifterState[i] = 1;
					}
				}
			} else {
				for (int i = 0; i < 3; i++) {
					if (getCachedColor(i) == color && !isLaunched) {
						isLaunched = true;
						lifterState[i] = 1;
					}
				}
			}
		}
		public void lifterUpdater() {
				for (int i = 0; i < 3; i++) {
					// Reverse Map for lifters
					boolean[] reverseMap = MDOConstants.LifterReverseMap;
					if (lifterState[i] == 1) {
						lifterState[i] = 2;
						lifterTimer.reset();
						lifter[i].setPosition(reverseMap[i] ? 1 - MDOConstants.LifterPositionHigh : MDOConstants.LifterPositionHigh); // reversed Boolean ? flipped 0 to 1 range: normal
					}
					else if (lifterState[i] == 2 && lifterTimer.milliseconds() > MDOConstants.LifterWaitToTopTimerMillis) {
						lifterState[i] = 0;
						lifter[i].setPosition(reverseMap[i] ? 1 - MDOConstants.LifterPositionLow : MDOConstants.LifterPositionLow); // reversed Boolean ? flipped 0 to 1 range: normal
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

		private CustomColor calcColor(int argb) {
			// Extract color components from ARGB int
			int red = (argb >> 16) & 0xFF;
			int green = (argb >> 8) & 0xFF;
			int blue = argb & 0xFF;

			if (green > blue && green > red) {
				return CustomColor.GREEN;
			} else if (blue > green && blue > red) {
				return CustomColor.PURPLE;
			} else {
				return CustomColor.NULL;
			}
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
		// (Think of it like a thermostat - how quickly it reacts to being off-target)
		double[] pidCoefficients;
		CustomPIDFController pidController;
		
		// TARGET: Where we want the servo to be (in degrees, 0-360)
		private volatile double targetPosition;

		// POSITION TRACKING: The servo position wraps at 360 degrees (full circle)
		// This prevents cord tangling by not tracking multiple rotations

		// The maximum reading from the sensor (360 degrees = full circle)
		private static final double WRAP_THRESHOLD = 360.0;

		// The current position we read from the sensor (0-360 degrees)
		private volatile double currentPosition = 0.0;

		// The last power level we sent to the servos (-1 to 1, where 0 is stopped)
		private volatile double lastAppliedPower = 0.0;
		// Cached simple mode position to prevent duplicate writes
		private volatile double lastSimplePosition = Double.NaN;

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
			// This sets our "zero point" so we know where we're starting from
			if (useAnalogPositionSensors && aPosition != null) {
				try {
					// Read the sensor and convert voltage to degrees (0-360)
					currentPosition = voltageToDegrees(aPosition.getVoltage());

					// Set our target to current position (don't move yet)
					targetPosition = currentPosition;
				} catch (Exception e) {
					// If sensor reading fails, start at safe zero position
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
			synchronized (lock) {
				if (!useAnalog) {
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
				} else {
					// SMART MODE: Set the target position (the PID loop will move the servo there)
					
					// Convert from our range [-1, 1] to degrees [0, 180]
					targetPosition = (position + 1.0) / 2.0 * WRAP_THRESHOLD;
				}
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
		 * The servo position is strictly kept between 0-360 degrees.
		 * The controller will NEVER cross the 0/360 boundary to reach a target.
		 * This prevents cord tangling by ensuring the servo always unwinds
		 * rather than spinning continuously in one direction.
		 */
		public void servoPidLoop() {
			// Only run this if we're in smart mode with a position sensor
			if (useAnalog && aPosition != null) {
				synchronized (lock) {
					try {
						// STEP 1: Read the sensor and convert voltage to degrees (0-360)
						currentPosition = voltageToDegrees(aPosition.getVoltage());

						// STEP 2: Use the PID controller to calculate how much power we need
						// We pass a large scale (WRAP_THRESHOLD + 100) to disable the PID's built-in
						// shortest-path logic. This ensures the servo respects the 0 and 360 boundaries
						// and doesn't cross the "gap" which could tangle wires.
						double power = pidController.calculate(targetPosition, currentPosition, 0, 2, WRAP_THRESHOLD + 100.0);

						// STEP 4: Make sure power stays within safe limits (-1 to 1)
						power = Math.max(-1.0, Math.min(1.0, power));

						// STEP 5: If sensor direction is backwards, flip the power direction
						if (invertServoDirection) {
							power = -power;
						}

						// Remember what power we applied (useful for debugging)
						lastAppliedPower = power;

						// STEP 6: Send power to each servo in the group
						for (int i = 0; i < servoGroup.length; i++) {
							String servoName = servoGroup[i];
							Servo s = this.servo.get(servoName);
							if (s != null) {
								// If this specific servo is reversed, flip its power
								double finalPower = reverseMap[i] ? -power : power;
								
								// Convert power from [-1, 1] to servo range [0, 1]
								double mapped = (finalPower + 1.0) / 2.0;
								mapped = Math.max(0.0, Math.min(1.0, mapped)); // Safety clamp

								// Send the command to the servo
								s.setPosition(mapped);
							}
						}
					} catch (Exception e) {
						// If anything goes wrong (sensor error, etc.), don't crash - just print error
						System.err.println("Error in servo PID loop: " + e.getMessage());
					}
				}
			}
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
					// Return current position (0-360 degrees, no accumulation)
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
		return targetPosition;  // volatile read is atomic
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
}
}
