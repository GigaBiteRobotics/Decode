package org.firstinspires.ftc.teamcode.drive;

public class LocalizationAutoAim {

	/**
	 * Calculates the required launch angle (in radians) to hit a target from a given position,
	 * using projectile motion equations.
	 *
	 * @param launchXYZ Double array representing the \[x, y, z\] coordinates of the launch position
	 * @param targetXYZ Double array representing the \[x, y, z\] coordinates of the target position
	 * @param v initial launch velocity (m/s)
	 * @param g acceleration due to gravity (m/s^2)
	 * @return the launch elevation and azimuth angles in radians or null if the target is unreachable
	 */
	public static Double[] calculateLaunchAngle(
			Double[] launchXYZ,
			Double[] targetXYZ,
			Double v, Double g) {
		Double vertDist = targetXYZ[2] - launchXYZ[2];
		Double horizontalDist = Math.hypot(targetXYZ[0] - launchXYZ[0], targetXYZ[1] - launchXYZ[1]);
		Double[] targetXY = {horizontalDist, vertDist};
		Double[] launchXY = {0.0, 0.0};
		Double elevation = calculateLaunchElevation(launchXY, targetXY, v, g);
		if (elevation == null) {
			return null; // Target unreachable
		}
		double azimuth = Math.atan2(targetXYZ[1] - launchXYZ[1], targetXYZ[0] - launchXYZ[0]);
		return new Double[]{elevation, azimuth};
	}
	public static Double calculateLaunchElevation(Double[] launchXY, Double[] targetXY, Double v, Double g) {
		Double dx = targetXY[0] - launchXY[0];
		Double dy = targetXY[1] - launchXY[1];
		Double part = (Math.pow(v, 4) - g * (g * (dx * dx + dy * dy) + 2 * (targetXY[2] - launchXY[2]) * Math.pow(v, 2)));
		if (part <= 0) {
			return null; // Target unreachable
		} else if (dx <= 0) return null; // Prevent backward shots
		else {
			Double angle1 = Math.atan((Math.pow(v, 2) + Math.sqrt(part)) / (g * dx));
			Double angle2 = Math.atan((Math.pow(v, 2) - Math.sqrt(part)) / (g * dx));
			return Math.min(angle1, angle2); // Return the lower angle for a more direct shot
		}
	}
}
