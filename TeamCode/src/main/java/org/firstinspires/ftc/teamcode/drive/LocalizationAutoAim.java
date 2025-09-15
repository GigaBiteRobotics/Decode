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
		Double horizDist = Math.hypot(targetXYZ[0] - launchXYZ[0], targetXYZ[1] - launchXYZ[1]);
		Double elevation = calculateLaunchElevation(horizDist, vertDist, v, g);
		if (elevation == null) {
			return null; // Target unreachable
		}
		double azimuth = Math.atan2(targetXYZ[1] - launchXYZ[1], targetXYZ[0] - launchXYZ[0]);
		return new Double[]{elevation, azimuth};
	}
	public static Double calculateLaunchElevation(Double x, Double y, Double v, Double g) {
		Double v2 = v * v;
		Double gx2 = g * x * x;
		Double twoYv2 = 2 * y * v2;
		double discriminant = v2 * v2 - g * (gx2 + twoYv2);

		if (discriminant < 0) {
			return null; // Target unreachable
		}

		Double sqrtDisc = Math.sqrt(discriminant);
		return Math.atan((v2 - sqrtDisc) / (g * x));
	}
}
