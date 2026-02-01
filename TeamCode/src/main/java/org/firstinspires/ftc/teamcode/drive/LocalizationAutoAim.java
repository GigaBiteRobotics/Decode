package org.firstinspires.ftc.teamcode.drive;

public class LocalizationAutoAim {

	/**
	 * Calculates the required launch angle (in radians) to hit a target from a given position,
	 * using projectile motion equations.
	 *
	 * @param launchXYZ [x, y, z] coordinates of the launch position
	 * @param targetXYZ [x, y, z] coordinates of the target position
	 * @param v initial launch velocity (length/s)
	 * @param g acceleration due to gravity (length/s^2)
	 * @return Double[] {elevation (radians), azimuth (radians)} or null if unreachable
	 */
	public static Double[] calculateLaunchAngle(
			Double[] launchXYZ,
			Double[] targetXYZ,
			Double v, Double g) {

		if (launchXYZ == null || targetXYZ == null || launchXYZ.length < 3 || targetXYZ.length < 3) {
			return null; // Invalid input
		}

		double dx = targetXYZ[0] - launchXYZ[0];
		double dy = targetXYZ[1] - launchXYZ[1];
		double dz = targetXYZ[2] - launchXYZ[2]; // Vertical difference (target - launch)

		double horizontalDist = Math.hypot(dx, dy);

		Double elevation = calculateLaunchElevation(horizontalDist);
		if (elevation == null) {
			return null; // Target unreachable
		}

		double azimuth = Math.atan2(dy, dx);
		return new Double[]{elevation, azimuth};
	}

	public static Double calculateLaunchElevation(double horizontalDist) {
		// get MDOConstants max and min
		Double maxElevation = MDOConstants.ElevationMax;
		Double maxElevationFT = MDOConstants.ElevationMaxIN;
		Double minElevation = MDOConstants.ElevationMin;
		Double minElevationFT = MDOConstants.ElevationMinIN;
		// map max elevation ft to max elevation, min elevation ft to min elevation
		// Convert horizontalDist (Inches) to Feet for calculation
		double horizontalDistFeet = horizontalDist / 12.0;

		double elevationRange = maxElevation - minElevation;
		double ftRange = maxElevationFT - minElevationFT;
		double elevation = minElevation + (horizontalDistFeet - minElevationFT) * elevationRange / ftRange;
		// clamp elevation to max and min
		if (elevation > maxElevation) {
			elevation = maxElevation;
		} else if (elevation < minElevation) {
			elevation = minElevation;
		}
		return elevation;
	}
	public static double getDistance(Double[] pos1, Double[] pos2) {
		if (pos1 == null || pos2 == null || pos1.length < 3 || pos2.length < 3) {
			return 0.0; // Invalid input
		}
		double dx = pos2[0] - pos1[0];
		double dy = pos2[1] - pos1[1];
		double dz = pos2[2] - pos1[2];
		return Math.sqrt(dx * dx + dy * dy + dz * dz);
	}
}