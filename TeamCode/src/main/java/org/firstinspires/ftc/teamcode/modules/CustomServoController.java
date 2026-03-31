package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.constants.MDOConstants;


import java.util.HashMap;

/**
 * CustomAxonServoController - A smart controller for servo motors with position feedback
 * <p>
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
	PIDFController pidfController;

	private volatile double targetPosition;
	private volatile double currentPosition = 0.0;
	private static final double WRAP_THRESHOLD = 360.0;
	private volatile double lastAppliedPower = 0.0;
	private volatile double lastSimplePosition = Double.NaN;
	private double lastEffectiveTarget = 0.0; // For tracking-aware slew rate
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

	public CustomServoController(HardwareMap hardwareMap, String[] servoGroup, boolean[] reverseMap, boolean useAnalogPositionSensors, double[] pidfCoefficients, String analogPositionName) {
		this.useAnalog = useAnalogPositionSensors;
		this.servo = new HashMap<>();
		this.servoGroup = servoGroup;
		this.reverseMap = reverseMap;

		this.pidfController = new PIDFController(pidfCoefficients[0], pidfCoefficients[1], pidfCoefficients[2], pidfCoefficients[3]);

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
				// Update PIDF coefficients if changed via dashboard
				setPIDFCoefficients(MDOConstants.AzimuthPIDFConstants);

				currentPosition = voltageToDegrees(aPosition.getVoltage());

				double effectiveTarget = targetPosition;

				// === PIDF calculation with real target & measurement ===
				// The PIDF controller output is a velocity command for continuous rotation servos.
				double power = pidfController.calculate(currentPosition, effectiveTarget);

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
				// Remove center offset when ForwardAimMode is enabled so the servo has
				// zero bias and aims straight forward with no adjustments applied.
				double centerOffset = MDOConstants.EnableForwardAimMode ? 0.0 : MDOConstants.AzimuthServoCenterOffset;
				double mapped = (finalPower + 1.0) / 2.0 + centerOffset;
				mapped = Math.max(0.0, Math.min(1.0, mapped));
				s.setPosition(mapped);
			}
		}
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
	 * <p>
	 * The servo driver's internal rail powers both the motor and the position sensor.
	 * Any motor current (regardless of direction) causes the rail to sag, which makes
	 * the ratiometric sensor output drop proportionally — the reading is always LOW
	 * under load, never high.
	 * <p>
	 * Physics model:
	 * V_rail_actual = V_rail_nominal × (1 - k × |power|)
	 * V_sensor      = position_fraction × V_rail_actual
	 * <p>
	 * Naive conversion (what we were doing):
	 * raw_deg = (V_sensor / V_adc_max) × 360
	 * <p>
	 * This is wrong under load because V_sensor is scaled down by the sag.
	 * The true position fraction is V_sensor / V_rail_actual, not V_sensor / V_adc_max.
	 * Since V_rail_actual = V_adc_max × (1 - k × |power|):
	 * <p>
	 * true_deg = raw_deg / (1 - k × |power|)
	 * <p>
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

	/**
	 * Get the last estimated voltage sag as a fraction (0.0 = no sag, 0.05 = 5% rail drop).
	 */
	public double getLastVoltageSag() {
		return lastSagFraction;
	}

	/**
	 * Get the last compensation applied in degrees (always ≥ 0, since sag always reads low).
	 */
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
			return pidfController.getPositionError();
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

	public void setPIDFCoefficients(double[] pidfCoefficients) {
		synchronized (lock) {
			if (pidfCoefficients.length < 4) {
				throw new IllegalArgumentException("PIDF coefficients array must have at least 4 elements: p, i, d, f.");
			}

			// Compare against the controller's actual current gains, not a stored array
			// reference — the dashboard may update the source array in-place.
			if (Math.abs(pidfController.getP() - pidfCoefficients[0]) < 0.0001 &&
					Math.abs(pidfController.getI() - pidfCoefficients[1]) < 0.0001 &&
					Math.abs(pidfController.getD() - pidfCoefficients[2]) < 0.0001 &&
					Math.abs(pidfController.getF() - pidfCoefficients[3]) < 0.0001) {
				return;
			}

			pidfController.setPIDF(pidfCoefficients[0], pidfCoefficients[1], pidfCoefficients[2], pidfCoefficients[3]);
		}
	}
}

