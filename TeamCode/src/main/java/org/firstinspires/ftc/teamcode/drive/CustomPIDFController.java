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
		// Calculate error
		error = targetPosition - currentPosition;

		// Calculate time delta
		long currentTime = System.currentTimeMillis();
		double deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds

		// Proportional term
		double pTerm = kP * error;

		// Integral term
		errorSum += error * deltaTime;
		double iTerm = kI * errorSum;
		if (currentPosition > targetPosition-range && currentPosition < targetPosition+range){
			errorSum = 0;
			iTerm = 0;
		}
		// Derivative term
		double derivative = (error - lastError) / deltaTime;
		double dTerm = kD * derivative;

		// Feedforward term
		double fTerm = kF * targetVelocity;

		// Save current state for next calculation
		lastError = error;
		lastTime = currentTime;

		// Calculate total output
		double output = pTerm + iTerm + dTerm + fTerm;

		// Clamp output to valid motor power range
		lastOutput = Math.min(1, Math.max(output/scale, -1)); // Scale and clamp
		return lastOutput;
	}

	public double getLastOutput() {
		return lastOutput;
	}
}