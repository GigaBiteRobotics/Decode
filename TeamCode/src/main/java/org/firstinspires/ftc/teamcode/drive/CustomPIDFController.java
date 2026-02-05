package org.firstinspires.ftc.teamcode.drive;

public class CustomPIDFController {
	// PIDF coefficients
	private final double kP;
	private final double kI;
	private final double kD;
	private final double kF;
	public volatile double error;

	// Internal state variables
	private double errorSum = 0;    // Integral term accumulator
	private double lastError = 0;  // For derivative term
	private double lastDerivative = 0; // For derivative filtering
	private long lastTime = System.currentTimeMillis(); // Time tracking
	private double lastOutput = 0; // Last calculated output

	// Constructor
	public CustomPIDFController(double kP, double kI, double kD, double kF) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
	}

	/**
	 * Calculates motor power based on target position and current position.
	 * @param targetPosition The desired encoder position.
	 * @param currentPosition The current encoder position.
	 * @param targetVelocity The desired feedforward velocity (ticks/sec). Use 0 if not required.
	 * @param range The range within which the error is considered tolerable.
	 * @return Calculated motor power (range: -1.0 to 1.0).
	 */
	public synchronized double calculate(double targetPosition, double currentPosition, double targetVelocity, double range) {
		return calculate(targetPosition, currentPosition, targetVelocity, range, 5000.0);
	}

	/**
	 * Calculates motor power based on target position and current position with custom scaling.
	 * @param targetPosition The desired encoder position.
	 * @param currentPosition The current encoder position.
	 * @param targetVelocity The desired feedforward velocity (ticks/sec). Use 0 if not required.
	 * @param range The range within which the error is considered tolerable.
	 * @param scale Scale factor to normalize output (use 5000 for motors, 180 for servos in degrees).
	 * @return Calculated motor power (range: -1.0 to 1.0).
	 */
	public synchronized double calculate(double targetPosition, double currentPosition, double targetVelocity, double range, double scale) {
		// Calculate error with wrap-around handling for angular positions
		// When scale is 180 (degrees), handle wrap-around to find shortest path
		error = targetPosition - currentPosition;

		// Wrap-around correction for angular positions (when scale is small like 180)
		if (scale <= 360.0) {  // Likely an angular measurement
			// Normalize error to [-scale/2, scale/2] to find shortest path
			while (error > scale / 2.0) {
				error -= scale;
			}
			while (error < -scale / 2.0) {
				error += scale;
			}
		}

		// Calculate time delta
		long currentTime = System.currentTimeMillis();
		double deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds

		// Prevent division by zero or negative time
		if (deltaTime <= 0.0) {
			deltaTime = 0.001; // 1ms minimum
		}

		// Deadzone: if error is very small, set output to zero to prevent jitter
		// Use configurable deadzone from MDOConstants for azimuth (when scale <= 360), otherwise use default 0.1
		double deadzonePercent = (scale <= 360.0) ? MDOConstants.AzimuthPIDDeadzonePercent : 0.1;
		double deadzone = range * deadzonePercent;
		if (Math.abs(error) < deadzone) {
			errorSum = 0; // Clear integral when in deadzone
			lastError = error;
			lastTime = currentTime;
			lastOutput = 0.0;
			return 0.0;
		}

		// Proportional term
		double pTerm = kP * error;

		// Integral term with anti-windup
		// Reset integral if error direction changed (crossed target = overshoot, or reversed direction)
		boolean errorDirectionChanged = (error > 0 && lastError < 0) || (error < 0 && lastError > 0);
		if (errorDirectionChanged) {
			errorSum = 0;  // Reset integral on any zero-crossing to prevent overshoot
			lastDerivative = 0; // Also reset derivative filter on direction change
		}

		errorSum += error * deltaTime;
		double iTerm = kI * errorSum;

		// Reset integral when within range
		if (Math.abs(error) < range) {
			errorSum = 0;
			iTerm = 0;
		}
		// Integral windup protection: clamp accumulated error
		double maxIntegral = scale * 0.5; // Max integral contribution
		if (Math.abs(errorSum) > maxIntegral) {
			errorSum = Math.signum(errorSum) * maxIntegral;
			iTerm = kI * errorSum;
		}

		// Derivative term with smoothing to reduce noise
		// Calculate error change, handling wrap-around for angular positions
		double errorChange = error - lastError;

		// For angular measurements, normalize the error change to prevent spikes at wrap-around
		if (scale <= 360.0) {
			while (errorChange > scale / 2.0) {
				errorChange -= scale;
			}
			while (errorChange < -scale / 2.0) {
				errorChange += scale;
			}
		}

		double derivative = errorChange / deltaTime;

		// Low-pass filter on derivative to reduce noise-induced jitter
		// This blends the current derivative with the previous one
		double filteredDerivative = derivative * 0.7 + lastDerivative * 0.3;
		lastDerivative = filteredDerivative; // Store for next iteration
		double dTerm = kD * filteredDerivative;

		// Feedforward term
		double fTerm = kF * targetVelocity;

		// Calculate total output
		double output = pTerm + iTerm + dTerm + fTerm;

		// Scale output to valid power range
		double scaledOutput = output / scale;

		// Clamp to valid power range
		scaledOutput = Math.min(1.0, Math.max(scaledOutput, -1.0));

		// Save current state for next calculation
		lastError = error;
		lastTime = currentTime;
		lastOutput = scaledOutput;

		return lastOutput;
	}

	public double getLastOutput() {
		return lastOutput;
	}

	/**
	 * Creates a copy of this PIDF controller with the same coefficients but fresh state.
	 * Useful when multiple motors need independent PID control with the same tuning.
	 * @return A new CustomPIDFController with the same kP, kI, kD, kF values.
	 */
	public CustomPIDFController copy() {
		return new CustomPIDFController(kP, kI, kD, kF);
	}
}