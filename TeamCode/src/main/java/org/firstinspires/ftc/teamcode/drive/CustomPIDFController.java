package org.firstinspires.ftc.teamcode.drive;

public class CustomPIDFController {
	// PIDF coefficients
	private final double kP;
	private final double kI;
	private final double kD;
	private final double kF;
	public double error;

	// Internal state variables
	private double errorSum = 0;    // Integral term accumulator
	private double lastError = 0;  // For derivative term
	private long lastTime = System.currentTimeMillis(); // Time tracking

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
	public double calculate(double targetPosition, double currentPosition, double targetVelocity, double range) {
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
		return Math.min(1, Math.max(output/5000, -1)); // Reverse logic if needed
	}
}