package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CustomMotor {
	private final com.qualcomm.robotcore.hardware.DcMotorEx motor;
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
	 * @return RPM value (primitive double)
	 */
	public double getRPM() {
		if (!hasEncoder) {
			throw new IllegalStateException("Encoder disabled for this motor: " + motor.getDeviceName());
		}

		synchronized (rpmCacheLock) {
			long currentTime = System.currentTimeMillis();
			if (currentTime - lastRPMUpdateTime < RPM_CACHE_TTL_MS) {
				return cachedRPM;
			}

			double velocityTPS = motor.getVelocity();
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
	 * @return RPM value (primitive double)
	 */
	public double updateAndGetRPM() {
		if (!hasEncoder) return 0.0;

		synchronized (rpmCacheLock) {
			double velocityTPS = motor.getVelocity();
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
			double currentRPM = updateAndGetRPM();
			double power = pidfController.calculate(targetRPM, currentRPM, 0, 50);
			power = Math.max(-1.0, Math.min(1.0, power));

			if (Math.abs(power - lastAppliedPower) > 0.005 || (power == 0 && lastAppliedPower != 0) || (power != 0 && lastAppliedPower == 0)) {
				motor.setPower(power);
				lastAppliedPower = power;
			}
		}
	}

	public void setPower(double power) {
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

	/**
	 * Check if this motor has an encoder enabled
	 * @return true if encoder is enabled
	 */
	public boolean hasEncoder() {
		return hasEncoder;
	}
}

