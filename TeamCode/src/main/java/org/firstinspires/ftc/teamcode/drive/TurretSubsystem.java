package org.firstinspires.ftc.teamcode.drive;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TurretSubsystem - Manages the azimuth (turret) servo, IMU-based heading correction,
 * auto-aim calculations, and manual azimuth offsets.
 */
public class TurretSubsystem {

	private final CustomAxonServoController azimuthServo;

	// State
	private double manualAzimuthOffset = 0.0;
	private Double launchAzimuthDeg = 0.0;
	private Double robotFieldRelativeAzimuthDeg = 0.0;
	private Double finalAzimuthDeg = 0.0;

	// Timers
	private final ElapsedTime azimuthAdjustTimer = new ElapsedTime();

	public TurretSubsystem(CustomAxonServoController azimuthServo) {
		this.azimuthServo = azimuthServo;
	}

	// ===== Simple actions for GamepadEventHandler callbacks =====

	/**
	 * Adjust the manual azimuth offset by the given amount (in degrees).
	 * @param deltaDeg Amount to add to the offset (positive = right, negative = left)
	 */
	public void adjustAzimuth(double deltaDeg) {
		manualAzimuthOffset += deltaDeg;
	}

	/** Reset the manual azimuth offset to zero. */
	public void resetAzimuthOffset() {
		manualAzimuthOffset = 0.0;
	}

	/**
	 * Handle D-pad input for manual azimuth adjustment.
	 * Call once per loop with the current gamepad state.
	 */
	public void handleInput(boolean dpadLeft, boolean dpadRight, boolean yButton) {
		if (azimuthAdjustTimer.milliseconds() > 150) {
			if (dpadRight) {
				manualAzimuthOffset += 1.0;
				azimuthAdjustTimer.reset();
			} else if (dpadLeft) {
				manualAzimuthOffset -= 1.0;
				azimuthAdjustTimer.reset();
			}
			if (yButton) {
				manualAzimuthOffset = 0.0;
				azimuthAdjustTimer.reset();
			}
		}
	}

	/**
	 * Run the aiming update for the turret.
	 * Calculates the turret servo position from heading, launch vectors, and offsets.
	 *
	 * @param currentPose   The current robot pose (thread-safe copy)
	 * @param launchVectors The launch vectors from LauncherCalculations (may be null)
	 * @param isRedSide     Whether the robot is on the red alliance
	 */
	public void update(Pose currentPose, Double[] launchVectors, boolean isRedSide) {
		robotFieldRelativeAzimuthDeg = Math.toDegrees(currentPose.getHeading());

		double azimuthFineAdjust = isRedSide ? MDOConstants.RedAzimuthFineAdjustment : MDOConstants.BlueAzimuthFineAdjustment;
		boolean hasValidLaunchVectors = (launchVectors != null);

		if (hasValidLaunchVectors) {
			launchAzimuthDeg = Math.toDegrees(launchVectors[1]);
		}

		if (MDOConstants.EnableTurretIMUCorrection && MDOConstants.EnableTurret) {
			if (MDOConstants.EnableLauncherCalcAzimuth && hasValidLaunchVectors) {
				double robotHeadingDeg = robotFieldRelativeAzimuthDeg;
				double targetAzimuthDeg = launchAzimuthDeg;

				finalAzimuthDeg = robotHeadingDeg - targetAzimuthDeg
						+ MDOConstants.AzimuthIMUOffset
						+ MDOConstants.AzimuthFineAdjustment
						+ azimuthFineAdjust
						+ manualAzimuthOffset;
			} else {
				double robotHeadingDeg = robotFieldRelativeAzimuthDeg;
				finalAzimuthDeg = (robotHeadingDeg + MDOConstants.AzimuthIMUOffset + manualAzimuthOffset)
						* MDOConstants.AzimuthMultiplier;
			}
		}

		// Normalize finalAzimuthDeg to [-180, 180] range
		while (finalAzimuthDeg > 180.0) {
			finalAzimuthDeg -= 360.0;
		}
		while (finalAzimuthDeg < -180.0) {
			finalAzimuthDeg += 360.0;
		}

		// Convert degrees to servo position [-1, 1] range
		double servoPosition = -finalAzimuthDeg / 180.0;

		// Update PID coefficients if changed via dashboard
		azimuthServo.setPIDCoefficients(MDOConstants.AzimuthPIDFConstants);

		// Set Azimuth Servos
		if (MDOConstants.EnableTurret) {
			azimuthServo.setPosition(servoPosition);
		} else {
			azimuthServo.setPosition(0.0);
		}
	}

	// ===== Getters for telemetry =====

	public double getManualAzimuthOffset() {
		return manualAzimuthOffset;
	}

	public Double getLaunchAzimuthDeg() {
		return launchAzimuthDeg;
	}

	public Double getRobotFieldRelativeAzimuthDeg() {
		return robotFieldRelativeAzimuthDeg;
	}

	public Double getFinalAzimuthDeg() {
		return finalAzimuthDeg;
	}

	public double getServoPosition() {
		return azimuthServo.getPosition();
	}

	public double getServoTarget() {
		return azimuthServo.getTargetPosition();
	}

	public CustomAxonServoController getServoController() {
		return azimuthServo;
	}
}

