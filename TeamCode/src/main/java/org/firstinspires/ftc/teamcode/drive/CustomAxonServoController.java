package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

/**
 * CustomAxonServoController - A smart controller for servo motors with position feedback
 *
 * Controls one or more servo motors in two ways:
 * 1. Simple Mode: Tell the servo where to go (like moving a dial)
 * 2. Smart Mode: Uses a sensor to know exactly where the servo is and automatically corrects its position
 */
public class CustomAxonServoController {
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
	private boolean allowWrapAround = true;
	private final Object lock = new Object();

	public CustomAxonServoController(HardwareMap hardwareMap, String[] servoGroup, boolean[] reverseMap, boolean useAnalogPositionSensors, double[] pidCoefficients, String analogPositionName) {
		this(hardwareMap, servoGroup, reverseMap, useAnalogPositionSensors, pidCoefficients, analogPositionName, false);
	}

	public CustomAxonServoController(HardwareMap hardwareMap, String[] servoGroup, boolean[] reverseMap, boolean useAnalogPositionSensors, double[] pidCoefficients, String analogPositionName, boolean invertServoDirection) {
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

				double error = targetPosition - currentPosition;

				if (allowWrapAround) {
					while (error > 180.0) error -= 360.0;
					while (error < -180.0) error += 360.0;

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

				double power = pidController.calculate(
						error, 0, 0, 2, 361.0);

				power = Math.max(-1.0, Math.min(1.0, power));

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

				double maxDelta = MDOConstants.AzimuthSlewRate;
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
		return (voltage / aPosition.getMaxVoltage()) * WRAP_THRESHOLD;
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

