package org.firstinspires.ftc.teamcode.drive.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class RedFarAutoConstants {
	public static Pose startPose = new Pose(97.8, 11.2, 0);
	public static Pose launchPose = new Pose(85, 16, 0);
	public static Pose ballCollection0LineupPose = new Pose(99, 37.5, 0);
	public static Pose ballCollection0PickupPose = new Pose(131, 37.5, 0);
	public static Pose ballCollection1LineupPose = new Pose(115, 13, 0);
	public static Pose ballCollection1PickupPose = new Pose(131, 13, 0);
	public static double elevationPos = -0.1;
	public static int targetRPM = 4300;
	public static double azimuthPos = -0.3;

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

	// Delay between shots in milliseconds
	public static int shotDelayMs = 1100;

	// Delay after launching before moving to next position in milliseconds
	public static int postLaunchDelayMs = 1100;

	// Time to wait for ball collection at each position in milliseconds
	public static int intakeCollect0TimeMs = 1800;
	public static int intakeCollect1TimeMs = 1800;

	// Time to linger at each ball position before moving to launch (milliseconds)
	public static int linger0TimeMs = 0;
	public static int linger1TimeMs = 0;
}
