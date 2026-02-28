package org.firstinspires.ftc.teamcode.drive;

import com.pedropathing.geometry.Pose;

/**
 * ElevationSubsystem - Manages the elevation servo, angle calculations,
 * clamping, and distance-based offset zones.
 */
public class ElevationSubsystem {

	private final CustomAxonServoController elevationServo;

	// State
	private Double launchElevationDeg = 0.0;
	private double elevationServoTarget = 0.0;
	private double elevationServoFinal = 0.0;

	public ElevationSubsystem(CustomAxonServoController elevationServo) {
		this.elevationServo = elevationServo;
	}

	/**
	 * Update the elevation servo based on launch vectors and distance offsets.
	 *
	 * @param currentPose   The current robot pose
	 * @param launchVectors The launch vectors from LauncherCalculations (may be null)
	 * @param isRedSide     Whether the robot is on the red alliance
	 */
	public void update(Pose currentPose, Double[] launchVectors, boolean isRedSide) {
		double elevationOffset = isRedSide ? MDOConstants.RedElevationOffset : MDOConstants.BlueElevationOffset;

		// Calculate distance-based elevation offset from zones
		Double[] currentTarget = isRedSide ? MDOConstants.redTargetLocation : MDOConstants.blueTargetLocation;
		double distanceElevationOffset = LauncherCalculations.calculateElevationOffset(currentPose, currentTarget, isRedSide);

		boolean hasValidLaunchVectors = (launchVectors != null);

		if (hasValidLaunchVectors) {
			// Elevation is already calculated as servo position in LocalizationAutoAim
			launchElevationDeg = launchVectors[0];
		}

		if (launchElevationDeg != null) {
			// Use alliance-specific elevation offset combined with shared offset and distance-based offset
			elevationServoTarget = (launchElevationDeg + MDOConstants.ElevationOffset + elevationOffset + distanceElevationOffset) * MDOConstants.ElevationMultiplier;
			// Clamp elevation to the configured min/max
			elevationServoFinal = Math.max(MDOConstants.ElevationClampMin, Math.min(MDOConstants.ElevationClampMax, elevationServoTarget));
			elevationServo.setPosition(elevationServoFinal);
		}
	}

	// ===== Getters for telemetry =====

	public Double getLaunchElevationDeg() {
		return launchElevationDeg;
	}

	public double getElevationServoFinal() {
		return elevationServoFinal;
	}

	public CustomAxonServoController getServoController() {
		return elevationServo;
	}
}

