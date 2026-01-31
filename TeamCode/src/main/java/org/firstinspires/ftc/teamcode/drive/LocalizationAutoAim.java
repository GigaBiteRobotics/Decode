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

		Double elevation = calculateLaunchElevation(horizontalDist, dz, v, g);
		if (elevation == null) {
			return null; // Target unreachable
		}

		double azimuth = Math.atan2(dy, dx);
		return new Double[]{elevation, azimuth};
	}

	/**
	 * Calculates the required launch elevation angle.
	 *
	 * Formula: theta = arctan( (v^2 ± sqrt(v^4 - g(gx^2 + 2yv^2))) / (gx) )
	 *
	 * @param x Horizontal distance
	 * @param y Vertical distance (height of target relative to launch)
	 * @param v Launch velocity
	 * @param g Gravity
	 * @return Angle in radians (smaller of two solutions), or null if unreachable
	 */
	public static Double calculateLaunchElevation(Double x, Double y, Double v, Double g) {
		if (x <= 1e-6) return null; // Avoid division by zero close to launcher

		double v2 = Math.pow(v, 2);
        double v4 = Math.pow(v, 4);
        double gx = g * x;

        // standard projectile motion discriminant: v^4 - g * (g*x^2 + 2*y*v^2)
        double discriminant = v4 - g * (g * x * x + 2 * y * v2);

		if (discriminant < 0) {
			return null; // Target unreachable (out of range)
		}

		double sqrtDisc = Math.sqrt(discriminant);

		// Calculate both possible ballistic trajectories
		double angle1 = Math.atan((v2 + sqrtDisc) / gx); // High arc (mortar)
		double angle2 = Math.atan((v2 - sqrtDisc) / gx); // Low arc (direct)

		// Return the lower angle (direct shot)
		return Math.min(angle1, angle2);
	}
}
