package org.firstinspires.ftc.teamcode.drive;

public class LocalizationAutoAim {
	public static Double calculateLaunchAngle(
			double x0, double y0, double z0,
			double x1, double y1, double z1,
			double v, double g) {
		double dx = x1 - x0;
		double dy = y1 - y0;
		double dz = z1 - z0;
		double d = Math.sqrt(dx * dx + dy * dy);

		double v2 = v * v;
		double underSqrt = v2 * v2 - g * (g * d * d + 2 * dz * v2);
		if (underSqrt < 0) {
			return null; // No solution
		}
		double sqrt = Math.sqrt(underSqrt);
		double angle1 = Math.atan((v2 + sqrt) / (g * d));
		double angle2 = Math.atan((v2 - sqrt) / (g * d));

		return Math.max(angle1, angle2); // Return the higher angle for a more arched trajectory
	}
}
