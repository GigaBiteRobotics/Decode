package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;

public class CustomMotorController {
	private final String[] motorGroup;
	HashMap<String, CustomMotor> motors;
	boolean[] reverseMap;
	boolean[] encoderReverseMap;
	boolean[] encoderEnableMap;
	private volatile int targetRPM = 0;
	private volatile double targetPower = 0.0;
	private boolean isRPMMode = false;

	// Single shared PIDF controller for group-level RPM control
	private CustomPIDFController groupPidfController;
	private double lastAppliedGroupPower = 123456.0;

	// Lock object for thread safety
	private final Object lock = new Object();

	/**
	 * Constructor for motor group controller (all motors same encoder setting)
	 */
	public CustomMotorController(HardwareMap hardwareMap, String[] motorGroup, boolean[] reverseMap, boolean hasEncoder, double ticksPerRev, CustomPIDFController pidfController) {
		this(hardwareMap, motorGroup, reverseMap, null, null, ticksPerRev, pidfController);
		this.encoderEnableMap = new boolean[motorGroup.length];
		for (int i = 0; i < motorGroup.length; i++) {
			this.encoderEnableMap[i] = hasEncoder;
		}
		for (int i = 0; i < motorGroup.length; i++) {
			String motorName = motorGroup[i];
			boolean reverseEnc = (encoderReverseMap != null) ? encoderReverseMap[i] : false;
			try {
				CustomMotor motor = new CustomMotor(hardwareMap, motorName, hasEncoder, ticksPerRev, null, reverseEnc);
				this.motors.put(motorName, motor);
			} catch (Exception e) {
				throw new IllegalArgumentException("Failed to initialize motor: " + motorName + " - " + e.getMessage());
			}
		}
	}

	/**
	 * Constructor for motor group controller with encoder reverse map
	 */
	public CustomMotorController(HardwareMap hardwareMap, String[] motorGroup, boolean[] reverseMap, boolean[] encoderReverseMap, boolean hasEncoder, double ticksPerRev, CustomPIDFController pidfController) {
		this(hardwareMap, motorGroup, reverseMap, encoderReverseMap, null, ticksPerRev, pidfController);
		this.encoderEnableMap = new boolean[motorGroup.length];
		for (int i = 0; i < motorGroup.length; i++) {
			this.encoderEnableMap[i] = hasEncoder;
		}
		for (int i = 0; i < motorGroup.length; i++) {
			String motorName = motorGroup[i];
			boolean reverseEnc = (encoderReverseMap != null) ? encoderReverseMap[i] : false;
			try {
				CustomMotor motor = new CustomMotor(hardwareMap, motorName, hasEncoder, ticksPerRev, null, reverseEnc);
				this.motors.put(motorName, motor);
			} catch (Exception e) {
				throw new IllegalArgumentException("Failed to initialize motor: " + motorName + " - " + e.getMessage());
			}
		}
	}

	/**
	 * Constructor for motor group controller with per-motor encoder enable map
	 */
	public CustomMotorController(HardwareMap hardwareMap, String[] motorGroup, boolean[] reverseMap, boolean[] encoderReverseMap, boolean[] encoderEnableMap, double ticksPerRev, CustomPIDFController pidfController) {
		this.motorGroup = motorGroup;
		this.reverseMap = reverseMap;
		this.encoderReverseMap = encoderReverseMap;
		this.encoderEnableMap = encoderEnableMap != null ? encoderEnableMap : new boolean[motorGroup.length];
		this.motors = new HashMap<>();
		this.groupPidfController = pidfController;

		if (motorGroup.length != reverseMap.length) {
			throw new IllegalArgumentException("Motor group and reverse map must have the same length.");
		}
		if (encoderReverseMap != null && motorGroup.length != encoderReverseMap.length) {
			throw new IllegalArgumentException("Motor group and encoder reverse map must have the same length.");
		}
		if (encoderEnableMap != null && motorGroup.length != encoderEnableMap.length) {
			throw new IllegalArgumentException("Motor group and encoder enable map must have the same length.");
		}

		for (int i = 0; i < motorGroup.length; i++) {
			String motorName = motorGroup[i];
			boolean reverseEnc = (encoderReverseMap != null) ? encoderReverseMap[i] : false;
			boolean hasEnc = this.encoderEnableMap[i];
			try {
				CustomMotor motor = new CustomMotor(hardwareMap, motorName, hasEnc, ticksPerRev, null, reverseEnc);
				this.motors.put(motorName, motor);
			} catch (Exception e) {
				throw new IllegalArgumentException("Failed to initialize motor: " + motorName + " - " + e.getMessage());
			}
		}
	}

	/**
	 * Set RPM for all motors in the group
	 */
	public void setRPM(int rpm) {
		synchronized (lock) {
			this.targetRPM = rpm;
			this.isRPMMode = true;
		}
	}

	/**
	 * Set power for all motors in the group
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
	 * Update RPM PID for all motors in the group - call this in a loop.
	 */
	public void updateRPMPID() {
		synchronized (lock) {
			if (isRPMMode && groupPidfController != null) {
				double currentRPM = getAverageRPMInternal();

				double power = groupPidfController.calculate(targetRPM, currentRPM, 0, 50);
				power = Math.max(-1.0, Math.min(1.0, power));

				if (Math.abs(power - lastAppliedGroupPower) > 0.005 || (power == 0 && lastAppliedGroupPower != 0) || (power != 0 && lastAppliedGroupPower == 0)) {
					for (int i = 0; i < motorGroup.length; i++) {
						String motorName = motorGroup[i];
						CustomMotor motor = motors.get(motorName);
						if (motor != null) {
							double finalPower = reverseMap[i] ? -power : power;
							motor.setRawPower(finalPower);
						}
					}
					lastAppliedGroupPower = power;
				}
			}
		}
	}

	/**
	 * Internal method to get average RPM without locking (caller must hold lock).
	 */
	private double getAverageRPMInternal() {
		double sum = 0.0;
		int count = 0;
		for (CustomMotor motor : motors.values()) {
			if (motor.hasEncoder()) {
				double rpm = motor.updateAndGetRPM();
				sum += rpm;
				count++;
			}
		}
		return count > 0 ? sum / count : 0.0;
	}

	/**
	 * Get average RPM of all motors in the group.
	 */
	public double getAverageRPM() {
		synchronized (lock) {
			double sum = 0.0;
			int count = 0;
			for (CustomMotor motor : motors.values()) {
				if (motor.hasEncoder()) {
					double rpm = motor.getRPM();
					sum += Math.abs(rpm);
					count++;
				}
			}
			return count > 0 ? sum / count : 0.0;
		}
	}

	/**
	 * Get RPM of a specific motor by name
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
	 * Set PIDF controller for group-level RPM control
	 */
	public void setPIDFController(CustomPIDFController pidfController) {
		synchronized (lock) {
			this.groupPidfController = pidfController;
		}
	}

	public int getTargetRPM() {
		return targetRPM;
	}

	public double getTargetPower() {
		return targetPower;
	}

	/**
	 * Get the last applied PID output power
	 */
	public double getPIDOutput() {
		synchronized (lock) {
			return lastAppliedGroupPower == 123456.0 ? 0.0 : lastAppliedGroupPower;
		}
	}

	public boolean isRPMMode() {
		return isRPMMode;
	}

	/**
	 * Get debug string with raw velocity values from all motors
	 */
	public String getDebugString() {
		StringBuilder sb = new StringBuilder();
		for (String motorName : motorGroup) {
			CustomMotor motor = motors.get(motorName);
			if (motor != null && motor.hasEncoder()) {
				double rawVel = motor.getRawVelocity();
				double rpm = motor.getRPM();
				double ticksPerRev = motor.getTicksPerRev();
				double expectedRPM = (ticksPerRev > 0) ? (rawVel / ticksPerRev) * 60.0 : 0;
				sb.append(String.format("%s: vel=%.0f rpm=%.1f exp=%.1f tpr=%.0f | ",
						motorName, rawVel, rpm, expectedRPM, ticksPerRev));
			}
		}
		return sb.toString();
	}
}

