package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.MDOConstants;

import java.util.HashMap;

/**
 * CustomAxonServoController - A smart controller for servo motors with position feedback
 *
 * Controls one or more servo motors in two ways:
 * 1. Simple Mode: Tell the servo where to go (like moving a dial)
 * 2. Smart Mode: Uses a sensor to know exactly where the servo is and automatically corrects its position
 */
public class CustomServoController {
	private final String[] servoGroup;
	HashMap<String, Servo> servo;
	AnalogInput aPosition;
	boolean useAnalog;
	boolean[] reverseMap;
	boolean invertServoDirection;
	double[] pidCoefficients;
	CustomPIDFController pidController;

	private volatile double targetPosition;
	private volatile double currentPosition = 0.0;
	private static final double WRAP_THRESHOLD = 360.0;
	private volatile double lastAppliedPower = 0.0;
	private volatile double lastSimplePosition = Double.NaN;
	private double lastEffectiveTarget = 0.0; // For tracking-aware slew rate
	private boolean allowWrapAround = true;
	private final Object lock = new Object();

	// === Internal voltage sag compensation ===
	// The servo driver's internal voltage rail powers both the motor and the analog
	// position sensor. When the motor draws current (any direction of movement),
	// the internal rail sags — always dropping, never rising. The analog sensor is
	// ratiometric to this rail, so its output drops proportionally:
	//
	//   V_sensor = (position_fraction) × V_rail
	//   V_rail   = V_nominal × (1 - k × |power|)
	//
	// where k = VoltageSagCompensationGain (tunable). Without compensation, we
	// compute degrees = (V_sensor / V_nominal_max) × 360, which reads LOW under load.
	// The fix: divide by the estimated sag to recover the true fraction:
	//
	//   true_degrees = raw_degrees / (1 - k × |power|)
	//
	// This is a multiplicative correction — the error scales with position (larger
	// reading = larger absolute error), which matches the ratiometric sensor physics.
	private volatile double lastSagFraction = 0.0;      // Last estimated sag (0.0 = none, 0.05 = 5% drop)
	private volatile double lastCompensationDeg = 0.0;   // Last correction applied (degrees)

	public CustomServoController(HardwareMap hardwareMap, String[] servoGroup, boolean[] reverseMap, boolean useAnalogPositionSensors, double[] pidCoefficients, String analogPositionName) {
		this(hardwareMap, servoGroup, reverseMap, useAnalogPositionSensors, pidCoefficients, analogPositionName, false);
	}

	public CustomServoController(HardwareMap hardwareMap, String[] servoGroup, boolean[] reverseMap, boolean useAnalogPositionSensors, double[] pidCoefficients, String analogPositionName, boolean invertServoDirection) {
		this.pidCoefficients = pidCoefficients;
		this.useAnalog = useAnalogPositionSensors;
		this.servo = new HashMap<>();
		this.servoGroup = servoGroup;
		this.reverseMap = reverseMap;
		this.invertServoDirection = invertServoDirection;

		this.pidController = new CustomPIDFController(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2], 0.0);

		if (useAnalogPositionSensors) {
			try {
				this.aPosition = hardwareMap.get(AnalogInput.class, analogPositionName);
			} catch (Exception e) {
				throw new IllegalArgumentException("Analog input with name " + analogPositionName + " not found in hardware map.");
			}
		}

		for (String servoName : servoGroup) {
			try {
				this.servo.put(servoName, hardwareMap.get(Servo.class, servoName));
			} catch (Exception e) {
				throw new IllegalArgumentException("Servo with name " + servoName + " not found in hardware map.");
			}
		}

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

	public void setPosition(double position) {
		if (!useAnalog) {
			synchronized (lock) {
				if (Math.abs(position - lastSimplePosition) < 0.001) {
					return;
				}
				lastSimplePosition = position;

				double mapped = (position + 1.0) / 2.0;
				mapped = Math.max(0.0, Math.min(1.0, mapped));

				for (String servoName : servoGroup) {
					Servo s = this.servo.get(servoName);
					if (s != null) {
						s.setPosition(mapped);
					}
				}
			}
		} else {
			double rawTargetDeg = (position + 1.0) / 2.0 * WRAP_THRESHOLD;

			if (allowWrapAround) {
				rawTargetDeg = normalizeDeg(rawTargetDeg);

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

			targetPosition = rawTargetDeg;
		}
	}

	public void stopServo() {
		synchronized (lock) {
			for (String servoName : servoGroup) {
				Servo s = this.servo.get(servoName);
				if (s != null) {
					s.setPosition(s.getPosition());
				}
			}
		}
	}

	public void servoPidLoop() {
		if (useAnalog && aPosition != null) {
			try {
				currentPosition = voltageToDegrees(aPosition.getVoltage());

				// === Forbidden zone escape (overrides normal PID) ===
				if (allowWrapAround) {
					double forbiddenCenter = MDOConstants.AzimuthForbiddenZoneCenter;
					double forbiddenHalfWidth = MDOConstants.AzimuthForbiddenZoneWidth / 2.0;

					if (forbiddenHalfWidth > 0 && isInForbiddenZone(currentPosition, forbiddenCenter, forbiddenHalfWidth)) {
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

				// === Compute effective target (adjusted for forbidden zone path avoidance) ===
				// The PID needs the actual target and measurement, not a pre-computed error,
				// so that derivative-on-measurement and target-tracking detection work correctly.
				double effectiveTarget = targetPosition;

				if (allowWrapAround) {
					// Check if the shortest path to the target crosses the forbidden zone.
					// If so, force the long way around by adjusting the effective target.
					double shortestError = targetPosition - currentPosition;
					while (shortestError > 180.0) shortestError -= 360.0;
					while (shortestError < -180.0) shortestError += 360.0;

					double forbiddenCenter = MDOConstants.AzimuthForbiddenZoneCenter;
					double forbiddenHalfWidth = MDOConstants.AzimuthForbiddenZoneWidth / 2.0;
					if (forbiddenHalfWidth > 0) {
						boolean shortestCrosses = pathCrossesForbiddenZone(
								currentPosition, currentPosition + shortestError, forbiddenCenter, forbiddenHalfWidth);
						if (shortestCrosses) {
							// Force the long way: offset the effective target so the PID
							// naturally takes the non-forbidden path.
							if (shortestError > 0) {
								effectiveTarget = currentPosition + (shortestError - 360.0);
							} else {
								effectiveTarget = currentPosition + (shortestError + 360.0);
							}
						}
					}
				}

				// === PID calculation with real target & measurement ===
				// The PID sees actual positions, enabling:
				// - Derivative-on-measurement (no kick when target moves)
				// - Target-moving detection (smooth tracking vs. static hold)
				double power = pidController.calculate(
						effectiveTarget, currentPosition, 0, 2, 361.0);

				power = Math.max(-1.0, Math.min(1.0, power));

				// === Dead band compensation (hardware-specific) ===
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

				if (invertServoDirection) {
					power = -power;
				}

				// === Tracking-aware slew rate limiter ===
				// When the target is moving (tracking), allow faster power changes so the
				// servo can keep up. When holding static, use the configured slew rate to
				// prevent oscillation/shaking.
				double maxDelta = MDOConstants.AzimuthSlewRate;
				double targetDelta = Math.abs(effectiveTarget - lastEffectiveTarget);
				if (targetDelta > 0.5) {
					// Target is moving — allow 3x faster power changes for responsive tracking
					maxDelta = Math.min(1.0, maxDelta * 3.0);
				}
				lastEffectiveTarget = effectiveTarget;

				double delta = power - lastAppliedPower;
				if (delta > maxDelta) {
					power = lastAppliedPower + maxDelta;
				} else if (delta < -maxDelta) {
					power = lastAppliedPower - maxDelta;
				}

				lastAppliedPower = power;

				applyPowerToServos(power);
			} catch (Exception e) {
				System.err.println("Error in servo PID loop: " + e.getMessage());
			}
		}
	}

	private void applyPowerToServos(double power) {
		for (int i = 0; i < servoGroup.length; i++) {
			String servoName = servoGroup[i];
			Servo s = this.servo.get(servoName);
			if (s != null) {
				double finalPower = reverseMap[i] ? -power : power;
				double mapped = (finalPower + 1.0) / 2.0 + MDOConstants.AzimuthServoCenterOffset;
				mapped = Math.max(0.0, Math.min(1.0, mapped));
				s.setPosition(mapped);
			}
		}
	}

	private double normalizeDeg(double deg) {
		deg = deg % 360.0;
		if (deg < 0) deg += 360.0;
		return deg;
	}

	private double shortestAngularDiff(double angle1, double angle2) {
		double diff = normalizeDeg(angle1) - normalizeDeg(angle2);
		while (diff > 180.0) diff -= 360.0;
		while (diff < -180.0) diff += 360.0;
		return diff;
	}

	private boolean pathCrossesPoint(double start, double end, double point) {
		while (start < 0) start += 360.0;
		while (start >= 360.0) start -= 360.0;
		while (point < 0) point += 360.0;
		while (point >= 360.0) point -= 360.0;

		double distance = end - start;
		if (Math.abs(distance) < 0.001) return false;

		boolean movingCW = distance > 0;

		if (movingCW) {
			double endNorm = start + Math.abs(distance);
			double pointCheck = point;
			if (point < start) pointCheck += 360.0;
			return pointCheck > start && pointCheck < endNorm;
		} else {
			double endNorm = start - Math.abs(distance);
			double pointCheck = point;
			if (point > start) pointCheck -= 360.0;
			return pointCheck < start && pointCheck > endNorm;
		}
	}

	private boolean isInForbiddenZone(double position, double center, double halfWidth) {
		while (position < 0) position += 360.0;
		while (position >= 360.0) position -= 360.0;
		double dist = angularDistance(position, center);
		return dist <= halfWidth;
	}

	private double angularDistance(double angle1, double angle2) {
		while (angle1 < 0) angle1 += 360.0;
		while (angle1 >= 360.0) angle1 -= 360.0;
		while (angle2 < 0) angle2 += 360.0;
		while (angle2 >= 360.0) angle2 -= 360.0;
		double diff = Math.abs(angle1 - angle2);
		return Math.min(diff, 360.0 - diff);
	}

	private boolean pathCrossesForbiddenZone(double start, double end, double forbiddenCenter, double halfWidth) {
		double forbiddenMin = forbiddenCenter - halfWidth;
		double forbiddenMax = forbiddenCenter + halfWidth;

		while (forbiddenMin < 0) forbiddenMin += 360.0;
		while (forbiddenMin >= 360.0) forbiddenMin -= 360.0;
		while (forbiddenMax < 0) forbiddenMax += 360.0;
		while (forbiddenMax >= 360.0) forbiddenMax -= 360.0;

		if (pathCrossesPoint(start, end, forbiddenCenter)) return true;
		if (pathCrossesPoint(start, end, forbiddenMin)) return true;
		if (pathCrossesPoint(start, end, forbiddenMax)) return true;
		return false;
	}

	public double getPosition() {
		synchronized (lock) {
			if (useAnalog) {
				return currentPosition;
			} else {
				throw new IllegalStateException("Analog position sensors are not enabled for this servo group.");
			}
		}
	}

	private double voltageToDegrees(double voltage) {
		return voltageToDegrees(voltage, lastAppliedPower);
	}

	/**
	 * Converts raw analog sensor voltage to degrees with internal voltage sag compensation.
	 *
	 * The servo driver's internal rail powers both the motor and the position sensor.
	 * Any motor current (regardless of direction) causes the rail to sag, which makes
	 * the ratiometric sensor output drop proportionally — the reading is always LOW
	 * under load, never high.
	 *
	 * Physics model:
	 *   V_rail_actual = V_rail_nominal × (1 - k × |power|)
	 *   V_sensor      = position_fraction × V_rail_actual
	 *
	 * Naive conversion (what we were doing):
	 *   raw_deg = (V_sensor / V_adc_max) × 360
	 *
	 * This is wrong under load because V_sensor is scaled down by the sag.
	 * The true position fraction is V_sensor / V_rail_actual, not V_sensor / V_adc_max.
	 * Since V_rail_actual = V_adc_max × (1 - k × |power|):
	 *
	 *   true_deg = raw_deg / (1 - k × |power|)
	 *
	 * k = VoltageSagCompensationGain, tuned empirically on the dashboard.
	 *
	 * @param voltage      Raw voltage from the position sensor
	 * @param appliedPower The servo power being commanded (-1 to 1)
	 * @return Position in degrees (0-360), compensated for internal rail sag
	 */
	private double voltageToDegrees(double voltage, double appliedPower) {
		double nominalMax = aPosition.getMaxVoltage();
		double rawDegrees = (voltage / nominalMax) * WRAP_THRESHOLD;

		double gain = MDOConstants.VoltageSagCompensationGain;
		if (gain <= 0.0) {
			// Compensation disabled
			lastSagFraction = 0.0;
			lastCompensationDeg = 0.0;
			return rawDegrees;
		}

		double absPower = Math.abs(appliedPower);
		double sagFraction = gain * absPower;  // e.g., 0.04 × 0.8 = 0.032 (3.2% sag)

		// Clamp so we never divide by zero or go negative
		if (sagFraction >= 0.99) sagFraction = 0.99;

		// Ratiometric correction: undo the multiplicative sag
		double compensatedDegrees = rawDegrees / (1.0 - sagFraction);

		lastSagFraction = sagFraction;
		lastCompensationDeg = compensatedDegrees - rawDegrees;

		return compensatedDegrees;
	}

	/** Get the last estimated voltage sag as a fraction (0.0 = no sag, 0.05 = 5% rail drop). */
	public double getLastVoltageSag() {
		return lastSagFraction;
	}

	/** Get the last compensation applied in degrees (always ≥ 0, since sag always reads low). */
	public double getLastCompensationDeg() {
		return lastCompensationDeg;
	}

	public double getRawPosition() {
		synchronized (lock) {
			if (useAnalog) {
				return currentPosition;
			} else {
				throw new IllegalStateException("Analog position sensors are not enabled for this servo group.");
			}
		}
	}

	public void resetPosition() {
		synchronized (lock) {
			currentPosition = voltageToDegrees(aPosition.getVoltage());
			targetPosition = currentPosition;
		}
	}

	public double getPIDTargetPower() {
		return lastAppliedPower;
	}

	public double getPIDError() {
		synchronized (lock) {
			return pidController.error;
		}
	}

	public double getTargetPosition() {
		return targetPosition;
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
		synchronized (lock) {
			if (pidCoefficients.length < 3) {
				throw new IllegalArgumentException("PID coefficients array must have at least 3 elements: p, i, d.");
			}

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

	public void setAllowWrapAround(boolean allow) {
		synchronized (lock) {
			this.allowWrapAround = allow;
		}
	}
}

