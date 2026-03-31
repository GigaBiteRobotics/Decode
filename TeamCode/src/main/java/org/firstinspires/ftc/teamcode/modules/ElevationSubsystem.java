package org.firstinspires.ftc.teamcode.modules;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.constants.MDOConstants;

/**
 * ElevationSubsystem - Manages the elevation servo, angle calculations,
 * clamping, and distance-based offset zones.
 */
public class ElevationSubsystem {

	private final CustomServoController elevationServo;

	// State
	private Double launchElevationDeg = 0.0;
	private double elevationServoTarget = 0.0;
	private double elevationServoFinal = 0.0;

	/** Manual elevation servo position used in Forward Aim Mode. */
	private double manualElevationOffset = MDOConstants.ForwardAimInitialElevation;

	public ElevationSubsystem(CustomServoController elevationServo) {
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
		// Forward Aim Mode: use the manually-adjusted servo position directly
		if (MDOConstants.EnableForwardAimMode) {
			elevationServoFinal = Math.max(MDOConstants.ElevationClampMin,
					Math.min(MDOConstants.ElevationClampMax, manualElevationOffset));
			elevationServo.setPosition(elevationServoFinal);
			return;
		}


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

	/**
	 * Adjust the manual elevation servo position (Forward Aim Mode only).
	 * Clamps the result to [ElevationClampMin, ElevationClampMax].
	 *
	 * @param delta Amount to add (servo position units, e.g. 0.02 per step)
	 */
	public void adjustManualElevation(double delta) {
		manualElevationOffset = Math.max(MDOConstants.ElevationClampMin,
				Math.min(MDOConstants.ElevationClampMax, manualElevationOffset + delta));
	}

	public double getManualElevationOffset() {
		return manualElevationOffset;
	}

	public double getElevationServoFinal() {
		return elevationServoFinal;
	}

	public CustomServoController getServoController() {
		return elevationServo;
	}
}
