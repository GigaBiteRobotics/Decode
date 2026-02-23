package org.firstinspires.ftc.teamcode.drive.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class BlueCloseAutoConstants {
 	public static Pose startPose = new Pose(22.85, 124.85, -0.71);
	public static Pose launchPose = new Pose(55, 90, 1.5708);
	public static Pose cameraLookPose = new Pose(55, 90, 1.5708); // Different heading for camera view
	public static Pose ballCollection0LineupPose = new Pose(48, 84, Math.PI);
	public static Pose ballCollection0PickupPose = new Pose(18, 83.5, Math.PI);
	public static Pose getBallCollection1LineupPose = new Pose(53, 61, Math.PI);
	public static Pose getBallCollection1PickupPose = new Pose(13, 61, Math.PI);
	public static Pose finalPose = new Pose(18, 90, Math.PI);
	public static double elevationPos = 0.12;
	public static int targetRPM = 3250;
	public static double azimuthPos = -0.43;

	// Intake speeds (0.0 to 1.0)
	public static double intakeInSpeed = 1;

	// Robot drive speeds for each path (0.0 to 1.0)
	public static double startToLaunchSpeed = 1;
	public static double launchToCameraLookSpeed = 1;
	public static double cameraLookToCollection0LineupSpeed = 1;
	public static double collection0LineupToPickupSpeed = 0.8;
	public static double collection0PickupToLaunchSpeed = 1;
	public static double launchToCollection1LineupSpeed = 1;
	public static double collection1LineupToPickupSpeed = 1;
	public static double collection1PickupToFinalSpeed = 0.9;

	// Delay between shots in milliseconds
	public static int shotDelayMs = 700;

	// Additional delay when launching a ball of a different color than the previous one
	public static int colorChangeDelayMs = 200;

	// Delay after launching before moving to next position in milliseconds
	public static int postLaunchDelayMs = 1000;

	// Time to wait for ball collection at each position in milliseconds
	public static int intakeCollect0TimeMs = 1800;
	public static int intakeCollect1TimeMs = 1800;

	// Time to linger at each ball position before moving to launch (milliseconds)
	public static int linger0TimeMs = 400;
	public static int linger1TimeMs = 0;

	// Maximum time in each launch state before moving on (prevents infinite loops)
	// 9 attempts (3 slots × 3 attempts) at ~1 second each = ~9 seconds, plus buffer
	public static long maxLaunchStateTimeMs = 12000;
}
