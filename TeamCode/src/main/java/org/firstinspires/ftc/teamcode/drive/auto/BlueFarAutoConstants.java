package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class BlueFarAutoConstants {
	// Mirrored from RedFarAutoConstants across x=72: X -> 144-X, Y same, heading -> Math.PI - heading
	public static Pose startPose = new Pose(144 - 97.8, 11.2, Math.PI);
	public static Pose launchPose = new Pose(144 - 85, 16, Math.PI);
	public static Pose ballCollection0LineupPose = new Pose(144 - 99, 37.5, Math.PI);
	public static Pose ballCollection0PickupPose = new Pose(144 - 131, 37.5, Math.PI);
	public static Pose ballCollection1LineupPose = new Pose(144 - 115, 13, Math.PI);
	public static Pose ballCollection1PickupPose = new Pose(144 - 131, 13, Math.PI);
	public static Pose finalPose = new Pose(144 - 100, 32, Math.PI);
	public static double elevationPos = -0.1;
	public static int targetRPM = 4300;
	public static double azimuthPos = 0.95;

	// Intake speeds (0.0 to 1.0)
	public static double intakeInSpeed = 1;

	// Robot drive speeds for each path (0.0 to 1.0)
	public static double startToLaunchSpeed = 1;
	public static double launchToCollection0LineupSpeed = 1;
	public static double collection0LineupToPickupSpeed = 1;
	public static double collection0PickupToLaunchSpeed = 1;
	public static double launchToCollection1LineupSpeed = 1;
	public static double collection1LineupToPickupSpeed = 1;
	public static double collection1PickupToFinalSpeed = 1;

	// Minimum percentage of target RPM required before firing (0.0 to 1.0)
	public static double rpmThresholdPercent = 0.9;

	// Minimum delay between shots in milliseconds (safety floor even when RPM is reached)
	public static int minShotDelayMs = 500;

	// Delay after launching before moving to next position in milliseconds
	public static int postLaunchDelayMs = 1100;

	// Time to wait for ball collection at each position in milliseconds
	public static int intakeCollect0TimeMs = 1800;
	public static int intakeCollect1TimeMs = 1800;

	// Time to linger at each ball position before moving to launch (milliseconds)
	public static int linger0TimeMs = 0;
	public static int linger1TimeMs = 500;

	// Auto time limit in seconds - when reached, interrupt everything and drive to final pose
	public static double autoTimeLimitSeconds = 28.0;
}



