package org.firstinspires.ftc.teamcode.drive;

import com.pedropathing.follower.Follower;

/**
 * Static utility class for launcher aiming and RPM calculations.
 * Extracted from RobotCoreCustom for modularity.
 */
public class LauncherCalculations {

	public static Double[] localizerLauncherCalc(Follower follower, Double[] target) {
		return localizerLauncherCalc(follower, target, true);
	}

	public static Double[] localizerLauncherCalc(Follower follower, Double[] target, boolean isRedSide) {
		if (follower == null || target == null || target.length < 3) {
			return null;
		}
		com.pedropathing.geometry.Pose currentPose = follower.getPose();
		if (currentPose == null) {
			return null;
		}
		return localizerLauncherCalc(currentPose, target, isRedSide);
	}

	/**
	 * Thread-safe overload that accepts a pre-fetched Pose
	 */
	public static Double[] localizerLauncherCalc(com.pedropathing.geometry.Pose pose, Double[] target, boolean isRedSide) {
		if (pose == null || target == null || target.length < 3) {
			return null;
		}
		Double[] poseArr = {pose.getX(), pose.getY(), pose.getHeading()};
		Double[] launcherConstants = isRedSide ? MDOConstants.RedLauncherCalcConstants : MDOConstants.BlueLauncherCalcConstants;
		return LocalizationAutoAim.calculateLaunchAngle(
				new Double[]{poseArr[0], poseArr[1], 10.0},
				new Double[]{target[0], target[1], target[2]},
				launcherConstants[0],
				launcherConstants[1]
		);
	}

	/**
	 * Calculate the appropriate launcher RPM based on distance to target
	 */
	public static int calculateLauncherRPM(Follower follower, Double[] target) {
		return calculateLauncherRPM(follower, target, true);
	}

	public static int calculateLauncherRPM(Follower follower, Double[] target, boolean isRedSide) {
		int staticRPM = isRedSide ? MDOConstants.RedLauncherRPM : MDOConstants.BlueLauncherRPM;

		if (follower == null) {
			return staticRPM;
		}
		com.pedropathing.geometry.Pose currentPose = follower.getPose();
		return calculateLauncherRPM(currentPose, target, isRedSide);
	}

	/**
	 * Thread-safe overload that accepts a pre-fetched Pose
	 */
	public static int calculateLauncherRPM(com.pedropathing.geometry.Pose pose, Double[] target, boolean isRedSide) {
		int staticRPM = isRedSide ? MDOConstants.RedLauncherRPM : MDOConstants.BlueLauncherRPM;
		double[][] rpmZones = isRedSide ? MDOConstants.RedLauncherRPMZones : MDOConstants.BlueLauncherRPMZones;

		if (!MDOConstants.EnableLauncherRPMZones) {
			return staticRPM;
		}

		if (pose == null || target == null || target.length < 3) {
			return staticRPM;
		}

		Double[] robotPos = {pose.getX(), pose.getY(), 10.0};
		double distance = LocalizationAutoAim.getDistance(robotPos, target);

		int rpm = staticRPM;

		for (int i = rpmZones.length - 1; i >= 0; i--) {
			if (distance >= rpmZones[i][0]) {
				rpm = (int) rpmZones[i][1];
				break;
			}
		}

		return rpm;
	}

	/**
	 * Calculate the elevation offset based on distance to target with alliance-specific tuning
	 */
	public static double calculateElevationOffset(Follower follower, Double[] target, boolean isRedSide) {
		if (follower == null) {
			return 0.0;
		}
		com.pedropathing.geometry.Pose currentPose = follower.getPose();
		return calculateElevationOffset(currentPose, target, isRedSide);
	}

	/**
	 * Thread-safe overload that accepts a pre-fetched Pose
	 */
	public static double calculateElevationOffset(com.pedropathing.geometry.Pose pose, Double[] target, boolean isRedSide) {
		double[][] elevationOffsetZones = isRedSide ? MDOConstants.RedElevationOffsetZones : MDOConstants.BlueElevationOffsetZones;

		if (pose == null || target == null || target.length < 3) {
			return 0.0;
		}

		Double[] robotPos = {pose.getX(), pose.getY(), 10.0};
		double distance = LocalizationAutoAim.getDistance(robotPos, target);

		double offset = 0.0;

		for (int i = elevationOffsetZones.length - 1; i >= 0; i--) {
			if (distance >= elevationOffsetZones[i][0]) {
				offset = elevationOffsetZones[i][1];
				break;
			}
		}

		return offset;
	}
}

