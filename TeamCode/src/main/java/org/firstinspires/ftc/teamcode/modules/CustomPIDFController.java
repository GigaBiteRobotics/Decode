package org.firstinspires.ftc.teamcode.modules;

import org.firstinspires.ftc.teamcode.constants.MDOConstants;

public class CustomPIDFController {
	// PIDF coefficients
	private final double kP;
	private final double kI;
	private final double kD;
	private final double kF;
	public volatile double error;

	// Internal state variables
	private double errorSum = 0;         // Integral term accumulator
	private double lastError = 0;        // For integral zero-crossing detection
	private double lastMeasurement = Double.NaN; // For derivative-on-measurement (avoids derivative kick)
	private double filteredDerivative = 0;       // Low-pass filtered derivative
	private long lastTimeNano = System.nanoTime();
	private double lastOutput = 0;
	private double lastTarget = Double.NaN;      // Track target changes for feedforward & deadzone

	// Constructor
	public CustomPIDFController(double kP, double kI, double kD, double kF) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
	}

	/**
	 * Calculates motor power based on target and current position.
	 *
	 * @param targetPosition  The desired position.
	 * @param currentPosition The current position.
	 * @param targetVelocity  The desired feedforward velocity. Use 0 if not required.
	 * @param range           The range within which the error is considered tolerable.
	 * @return Calculated power (range: -1.0 to 1.0).
	 */
	public synchronized double calculate(double targetPosition, double currentPosition, double targetVelocity, double range) {
		return calculate(targetPosition, currentPosition, targetVelocity, range, 5000.0);
	}

	/**
	 * Calculates motor power, optimized for both static hold and moving-target tracking.
	 * <p>
	 * Key design choices for moving targets:
	 * 1. Derivative-on-measurement: D term acts on -d(measurement)/dt, not d(error)/dt.
	 * This eliminates "derivative kick" — sudden D spikes when the target jumps/moves.
	 * 2. Integral doesn't reset on zero-crossing during tracking — only on direction
	 * reversal when the target is stationary (actual overshoot).
	 * 3. Deadzone only activates when the target is not moving — during tracking, any
	 * error produces output, preventing the stutter of repeated on/off engagement.
	 * 4. Target velocity feedforward: the F term can drive the servo ahead of the error,
	 * reducing tracking lag to near-zero for smooth movements.
	 *
	 * @param targetPosition  The desired position.
	 * @param currentPosition The current position (measurement).
	 * @param targetVelocity  Feedforward velocity (units/sec). Use 0 if not required.
	 * @param range           The range within which the error is considered tolerable.
	 * @param scale           Scale factor to normalize output (5000 for motors, 361 for servos in degrees).
	 * @return Calculated power (range: -1.0 to 1.0).
	 */
	public synchronized double calculate(double targetPosition, double currentPosition, double targetVelocity, double range, double scale) {
		boolean isAngular = (scale <= 360.0);
		double halfScale = scale / 2.0;

		// === Error calculation with wrap-around ===
		error = targetPosition - currentPosition;
		if (isAngular) {
			while (error > halfScale) error -= scale;
			while (error < -halfScale) error += scale;
		}

		// === Time delta ===
		long currentTimeNano = System.nanoTime();
		double deltaTime = (currentTimeNano - lastTimeNano) / 1.0e9;
		if (deltaTime <= 0.0) deltaTime = 0.000001;

		// === Detect if target is moving ===
		// Compare current target to previous target. If it changed, we're tracking.
		boolean targetMoving = false;
		if (!Double.isNaN(lastTarget)) {
			double targetDelta = targetPosition - lastTarget;
			if (isAngular) {
				while (targetDelta > halfScale) targetDelta -= scale;
				while (targetDelta < -halfScale) targetDelta += scale;
			}
			targetMoving = Math.abs(targetDelta) > 0.01;
		}

		// === Deadzone: only when target is stationary ===
		// During tracking (target moving), always produce output — even tiny errors
		// need correction, and dropping to 0 causes stutter.
		double deadzonePercent = isAngular ? MDOConstants.AzimuthPIDDeadzonePercent : 0.1;
		double deadzone = range * deadzonePercent;
		if (!targetMoving && Math.abs(error) < deadzone) {
			errorSum = 0;
			lastError = error;
			lastTarget = targetPosition;
			lastMeasurement = currentPosition;
			lastTimeNano = currentTimeNano;
			lastOutput = 0.0;
			return 0.0;
		}

		// === Proportional term ===
		double pTerm = kP * error;

		// === Integral term with tracking-aware anti-windup ===
		// Only reset on zero-crossing when the target is NOT moving.
		// During tracking, zero-crossings are normal (servo catching up then slightly
		// overshooting the moving target) and resetting the integral causes lag.
		boolean errorDirectionChanged = (error > 0 && lastError < 0) || (error < 0 && lastError > 0);
		if (errorDirectionChanged && !targetMoving) {
			errorSum = 0;
		}

		errorSum += error * deltaTime;

		// Windup clamp
		double maxIntegral = scale * 0.5;
		if (Math.abs(errorSum) > maxIntegral) {
			errorSum = Math.signum(errorSum) * maxIntegral;
		}

		double iTerm = kI * errorSum;

		// === Derivative term: derivative-on-measurement ===
		// Instead of d(error)/dt, compute -d(measurement)/dt.
		// This is mathematically equivalent when the target is constant, but when the
		// target changes, it ONLY responds to actual servo movement — no kick.
		double dTerm = 0.0;
		if (!Double.isNaN(lastMeasurement)) {
			double measurementChange = currentPosition - lastMeasurement;
			if (isAngular) {
				while (measurementChange > halfScale) measurementChange -= scale;
				while (measurementChange < -halfScale) measurementChange += scale;
			}
			double derivative = -measurementChange / deltaTime; // Negative: opposing movement

			// Low-pass filter (alpha=0.3 → 30% new, 70% old)
			filteredDerivative = 0.3 * derivative + 0.7 * filteredDerivative;
			dTerm = kD * filteredDerivative;
		}

		// === Feedforward term ===
		double fTerm = kF * targetVelocity;

		// === Total output ===
		double output = pTerm + iTerm + dTerm + fTerm;
		double scaledOutput = output / scale;
		scaledOutput = Math.min(1.0, Math.max(-1.0, scaledOutput));

		// === Save state ===
		lastError = error;
		lastTarget = targetPosition;
		lastMeasurement = currentPosition;
		lastTimeNano = currentTimeNano;
		lastOutput = scaledOutput;

		return lastOutput;
	}

	public double getLastOutput() {
		return lastOutput;
	}

	/**
	 * Creates a copy of this PIDF controller with the same coefficients but fresh state.
	 * Useful when multiple motors need independent PID control with the same tuning.
	 *
	 * @return A new CustomPIDFController with the same kP, kI, kD, kF values.
	 */
	public CustomPIDFController copy() {
		return new CustomPIDFController(kP, kI, kD, kF);
	}
}

