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

	public static class CustomMotor {
		private final DcMotor motor;
		private final double TICKS_PER_REV;
		private final boolean hasEncoder;
		private boolean isRPMMode = false;
		private int targetRPM = 0;
		private int lastTicks = 0;
		private double lastValidRPM = 0.0;
		private long lastTime = 0;
		private static final double MIN_DELTA_TIME = 0.1; // minimum 100ms between readings

		// PID coefficients
		private CustomPIDFController pidfController = new CustomPIDFController(0.1, 0.01, 0.005, 0.0);

		public CustomMotor(HardwareMap hardwareMap, String motorName, Boolean hasEncoder, double ticksPerRev, CustomPIDFController pidfController) {
			this.TICKS_PER_REV = ticksPerRev;
			this.pidfController = pidfController;
			try {
				this.motor = hardwareMap.get(DcMotor.class, motorName);
			} catch (Exception e) {
				throw new IllegalArgumentException("Motor with name " + motorName + " not found in hardware map.");
			}
			if (hasEncoder) {
				this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				this.hasEncoder = true;
			} else {
				this.hasEncoder = false;
			}
		}

		public Double getRPM() {
			if (!hasEncoder) {
				throw new IllegalStateException("Encoder disabled for this motor: " + motor.getDeviceName());
			}

			int currentTicks = motor.getCurrentPosition();
			long currentTime = System.nanoTime();

			// Initialize on first call
			if (lastTime == 0) {
				lastTicks = currentTicks;
				lastTime = currentTime;
				return lastValidRPM; // Will be 0.0 initially
			}

			double deltaTime = (currentTime - lastTime) / 1e9; // seconds

			// Only calculate if enough time has passed
			if (deltaTime < MIN_DELTA_TIME) {
				return lastValidRPM; // Return last valid reading
			}

			double deltaTicks = currentTicks - lastTicks;

			// Update stored values for next call
			lastTicks = currentTicks;
			lastTime = currentTime;

			// Calculate RPM
			double revs = deltaTicks / TICKS_PER_REV;
			double revsPerSec = revs / deltaTime;
			lastValidRPM = revsPerSec * 60.0;

			return lastValidRPM;
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
				double currentRPM = getRPM();
				double power = pidfController.calculate(targetRPM, currentRPM, 0, 5);
				power = Math.max(-1.0, Math.min(1.0, power)); // Clamp power to [-1, 1]
				motor.setPower(power);
			}

		}

		public void setPower(double power) {
			motor.setPower(power);
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
		}

		public CustomColor getColor(int pitSelector) {
			if (pitSelector < 0 || pitSelector >= 3) {
				throw new IllegalArgumentException("Invalid pit selector: " + pitSelector);
			}
			int colorARGB0 = colorSensor[pitSelector * 3].argb();
			CustomColor color0 = calcColor(colorARGB0);
			if (color0 != CustomColor.NULL) {
				return color0;
			} else {
				int colorARGB1 = colorSensor[pitSelector * 3 + 1].argb();
				return calcColor(colorARGB1);
			}
		}

		public void launch(CustomColor color) {
			if (color == CustomColor.NULL) {
				for (int i = 0; i < 3; i++) {
					if (getColor(i) == CustomColor.NULL) {
						lifterState[i] = 1;
					}
				}
			} else {
				for (int i = 0; i < 3; i++) {
					if (getColor(i) == color) {
						lifterState[i] = 1;
					}
				}
			}
		}
		public void lifterUpdater() {
				for (int i = 0; i < 3; i++) {
					if (lifterState[i] == 1) {
						lifterState[i] = 2;
						lifterTimer.reset();
						lifter[i].setPosition(MDOConstants.LifterPositionHigh);
					}
					if (lifterState[i] == 2 && lifterTimer.milliseconds() > MDOConstants.LifterWaitToTopTimerMillis) {
						lifterState[i] = 0;
						lifter[i].setPosition(MDOConstants.LifterPositionLow);
					}
					if (lifterState[i] != 0) {
						lifter[i].setPosition(MDOConstants.LifterPositionLow);
					}
				}
		}
		public void lightingUpdater() {
			for (int i = 0; i < 3; i++) {
				CustomColor color = calcColor(colorSensor[i].argb());
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

			if (green > blue * 1.5 && green > red * 1.5) {
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
	public static class CustomAxonServoController {
		private final String[] servoGroup;
		HashMap<String, Servo> servo;
		AnalogInput aPosition;
		boolean useAnalog;
		boolean[] reverseMap;
		boolean invertServoDirection; // If true, inverts PID output (when sensor direction is opposite to servo)
		double[] pidCoefficients;
		CustomPIDFController pidController;
		double targetPosition;

		// Wrap-around tracking for 180-degree sensors
		private static final double WRAP_THRESHOLD = 180.0; // Degrees where wrap occurs
		private double lastRawPosition = 0.0; // Last raw reading in degrees
		private int wrapCount = 0; // Number of full rotations
		private double accumulatedPosition = 0.0; // Total position including wraps
		private double lastAppliedPower = 0.0; // The actual power sent to servos after all transformations

		/**
		 *
		 * @param hardwareMap HardwareMap to get servos and analog inputs
		 * @param servoGroup Array of servo names in the group
		 * @param reverseMap Array of booleans indicating if each servo is reversed
		 * @param useAnalogPositionSensors Whether to use analog position sensors for feedback
		 * @param pidCoefficients PID coefficients for position control (if using analog sensors) - p, i, d, f, range
		 * @param analogPositionName Name of the analog position sensor
		 */
		public CustomAxonServoController(HardwareMap hardwareMap, String[] servoGroup, boolean[] reverseMap, boolean useAnalogPositionSensors, double[] pidCoefficients, String analogPositionName) {
			this(hardwareMap, servoGroup, reverseMap, useAnalogPositionSensors, pidCoefficients, analogPositionName, false);
		}

		/**
		 * Full constructor with servo direction inversion option
		 * @param hardwareMap HardwareMap to get servos and analog inputs
		 * @param servoGroup Array of servo names in the group
		 * @param reverseMap Array of booleans indicating if each servo is reversed
		 * @param useAnalogPositionSensors Whether to use analog position sensors for feedback
		 * @param pidCoefficients PID coefficients for position control (if using analog sensors) - p, i, d, f, range
		 * @param analogPositionName Name of the analog position sensor
		 * @param invertServoDirection If true, inverts the PID output (use when sensor reads opposite to servo movement)
		 */
		public CustomAxonServoController(HardwareMap hardwareMap, String[] servoGroup, boolean[] reverseMap, boolean useAnalogPositionSensors, double[] pidCoefficients, String analogPositionName, boolean invertServoDirection) {
			this.aPosition = hardwareMap.get(AnalogInput.class, analogPositionName);
			this.pidCoefficients = pidCoefficients;
			this.useAnalog = useAnalogPositionSensors;
			this.servo = new HashMap<>();
			this.servoGroup = servoGroup;
			this.reverseMap = reverseMap;
			this.invertServoDirection = invertServoDirection;
			this.pidController = new CustomPIDFController(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2], 0.0);
			for (String servoName : servoGroup) {
				try {
					this.servo.put(servoName, hardwareMap.get(Servo.class, servoName));
				} catch (Exception e) {
					throw new IllegalArgumentException("Servo with name " + servoName + " not found in hardware map, or its analog position sensor is missing if enabled.");
				}
			}

			// Initialize position tracking if using analog sensors
			if (useAnalogPositionSensors) {
				lastRawPosition = voltageToDegrees(aPosition.getVoltage());
				accumulatedPosition = lastRawPosition;
				targetPosition = lastRawPosition; // Start with target at current position
				wrapCount = 0;
			}
		}


		/**
		 * Sets the position of all servos in the group.
		 * @param position Desired position in range [-1, 1] - Unlike standard servo position [0, 1]
		 */

		public void setPosition(double position) {
			if (!useAnalog) {
				// Map input range [-1, 1] to servo range [0, 1]
				double mapped = (position + 1.0) / 2.0;
				mapped = Math.max(0.0, Math.min(1.0, mapped)); // clamp to [0,1]
				for (String servoName : servoGroup) {
					Servo s = this.servo.get(servoName);
					if (s != null) {
						s.setPosition(mapped);
					}
				}
			} else {
				// Map [-1, 1] to degrees range [0, 180]
				targetPosition = (position + 1.0) / 2.0 * WRAP_THRESHOLD;
			}
		}
		public void servoPidLoop() {
			if (useAnalog) {
				// Get raw sensor reading and convert to degrees (0-180)
				double rawDegrees = voltageToDegrees(aPosition.getVoltage());

				// Detect wrap-around
				double delta = rawDegrees - lastRawPosition;

				// If we jumped from near 180 to near 0, we wrapped forward
				if (delta < -WRAP_THRESHOLD / 2) {
					wrapCount++;
				}
				// If we jumped from near 0 to near 180, we wrapped backward
				else if (delta > WRAP_THRESHOLD / 2) {
					wrapCount--;
				}

				// Calculate accumulated position (includes wrap count)
				accumulatedPosition = rawDegrees + (wrapCount * WRAP_THRESHOLD);
				lastRawPosition = rawDegrees;


			// Use accumulated position for PID control with scale of 180 for degrees
			double power = pidController.calculate(targetPosition, accumulatedPosition, 0, 2, WRAP_THRESHOLD);

			power = Math.max(-1.0, Math.min(1.0, power)); // Clamp power to [-1, 1]

			// Invert power if sensor direction is opposite to servo direction
			if (invertServoDirection) {
				power = -power;
			}

			// Store the power being applied (before per-servo reversal)
			lastAppliedPower = power;

			for (int i = 0; i < servoGroup.length; i++) {
					String servoName = servoGroup[i];
					Servo s = this.servo.get(servoName);
					if (s != null) {
						double finalPower = reverseMap[i] ? -power : power;
						double mapped = (finalPower + 1.0) / 2.0;
						mapped = Math.max(0.0, Math.min(1.0, mapped)); // clamp to [0,1]
						s.setPosition(mapped);
					}
				}
			}
		}
		public double getPosition() {
			if (useAnalog) {
				// Return accumulated position (can be > 180 degrees)
				return accumulatedPosition;
			} else {
				throw new IllegalStateException("Analog position sensors are not enabled for this servo group.");
			}
		}

		/**
		 * Helper method to convert analog voltage to degrees (0-180 range)
		 */
		private double voltageToDegrees(double voltage) {
			// Assuming voltage maps linearly from 0V=0° to maxVoltage=180°
			return (voltage / aPosition.getMaxVoltage()) * WRAP_THRESHOLD;
		}

		/**
		 * Get the raw position (0-180 degrees) without accumulated wraps
		 */
		public double getRawPosition() {
			if (useAnalog) {
				return lastRawPosition;
			} else {
				throw new IllegalStateException("Analog position sensors are not enabled for this servo group.");
			}
		}

		/**
		 * Reset the wrap counter and accumulated position
		 */
		public void resetPosition() {
		wrapCount = 0;
		accumulatedPosition = voltageToDegrees(aPosition.getVoltage());
		lastRawPosition = accumulatedPosition;
	}
	public double getPIDTargetPower() {
		return lastAppliedPower;
	}
	public double getPIDError() {
		return pidController.error;
	}
	public double getTargetPosition() {
		return targetPosition;
	}
	public double getAccumulatedPosition() {
		return accumulatedPosition;
	}
	public double getAnalogVoltage() {
		if (useAnalog && aPosition != null) {
			return aPosition.getVoltage();
		}
		return 0.0;
	}
	public double getMaxVoltage() {
		if (useAnalog && aPosition != null) {
			return aPosition.getMaxVoltage();
		}
		return 0.0;
	}
	public void setPIDCoefficients(double[] pidCoefficients) {
		if (pidCoefficients.length < 3) {
			throw new IllegalArgumentException("PID coefficients array must have at least 3 elements: p, i, d.");
		}

		this.pidCoefficients = pidCoefficients;
		this.pidController = new CustomPIDFController(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2], 0.0);
	}
}
}