package org.firstinspires.ftc.teamcode.modules;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.constants.MDOConstants;

/**
 * ElevationSubsystemV2 — Elevation servo management with RPM-aware physics correction.
 * Extends the V1 approach by accepting the current launcher RPM so the elevation
 * angle can be fine-tuned for the actual muzzle velocity rather than a fixed value.
 */
public class ElevationSubsystemV2 {

    private final CustomServoController elevationServo;

    // State
    private Double launchElevationDeg = 0.0;
    private double elevationServoTarget = 0.0;
    private double elevationServoFinal = 0.0;

    /** Current launcher RPM supplied each loop for physics-based correction. */
    private double currentRPM = 0.0;

    /** Manual elevation servo position used in Forward Aim Mode. */
    private double manualElevationOffset = MDOConstants.ForwardAimInitialElevation;

    public ElevationSubsystemV2(CustomServoController elevationServo) {
        this.elevationServo = elevationServo;
    }

    /**
     * Supply the current launcher RPM so elevation can be adjusted for muzzle velocity.
     * Call once per loop before {@link #update}.
     */
    public void setCurrentRPM(double rpm) {
        this.currentRPM = rpm;
    }

    /**
     * Update the elevation servo based on launch vectors, distance offsets, and current RPM.
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

        // Optional RPM-based correction: scales offset when RPM deviates from nominal
        double nominalRPM = isRedSide ? MDOConstants.RedLauncherRPM : MDOConstants.BlueLauncherRPM;
        double rpmCorrectionDeg = 0.0;
        if (currentRPM > 100 && nominalRPM > 0) {
            // Small linear correction: more RPM → flatter angle, less RPM → steeper angle
            rpmCorrectionDeg = (nominalRPM - currentRPM) * MDOConstants.ElevationRPMCorrectionFactor;
        }

        boolean hasValidLaunchVectors = (launchVectors != null);

        if (hasValidLaunchVectors) {
            launchElevationDeg = launchVectors[0];
        }

        if (launchElevationDeg != null) {
            elevationServoTarget = (launchElevationDeg + MDOConstants.ElevationOffset + elevationOffset
                    + distanceElevationOffset + rpmCorrectionDeg) * MDOConstants.ElevationMultiplier;
            elevationServoFinal = Math.max(MDOConstants.ElevationClampMin,
                    Math.min(MDOConstants.ElevationClampMax, elevationServoTarget));
            elevationServo.setPosition(elevationServoFinal);
        }
    }

    /**
     * Adjust the manual elevation servo position (Forward Aim Mode only).
     *
     * @param delta Amount to add (servo position units, e.g. 0.02 per step)
     */
    public void adjustManualElevation(double delta) {
        manualElevationOffset = Math.max(MDOConstants.ElevationClampMin,
                Math.min(MDOConstants.ElevationClampMax, manualElevationOffset + delta));
    }

    // ===== Getters =====

    public Double getLaunchElevationDeg() {
        return launchElevationDeg;
    }

    public double getElevationServoFinal() {
        return elevationServoFinal;
    }

    public double getManualElevationOffset() {
        return manualElevationOffset;
    }

    public double getCurrentRPM() {
        return currentRPM;
    }

    public CustomServoController getServoController() {
        return elevationServo;
    }
}
