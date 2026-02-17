package org.firstinspires.ftc.teamcode.drive.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class RedCloseAutoConstants {
	// Mirrored from BlueCloseAutoConstants around x=72: X -> 144-X, Y same, heading -> π-heading
	public static Pose startPose = new Pose(144 - 22.85, 124.85, Math.PI - (-0.71));
	public static Pose launchPose = new Pose(144 - 55, 90, Math.PI - 1.5708);
	public static Pose cameraLookPose = new Pose(89, 90, 1.9544); // Different heading for camera view
	public static Pose ballCollection0LineupPose = new Pose(96, 86, 0);
	public static Pose ballCollection0PickupPose = new Pose(128, 86, 0);
	public static Pose getBallCollection1LineupPose = new Pose(91, 60, 0);
	public static Pose getBallCollection1PickupPose = new Pose(135, 59, 0);
	public static Pose finalPose = new Pose(144 - 18, 90, 0);
	public static double elevationPos = 0.12;
	public static int targetRPM = 3300;
	public static double azimuthPos = 1;

	// Intake speeds (0.0 to 1.0)
	public static double intakeInSpeed = 1;

	// Robot drive speeds for each path (0.0 to 1.0)
	public static double startToLaunchSpeed = 1;
	public static double launchToCameraLookSpeed = 1;
	public static double cameraLookToCollection0LineupSpeed = 1;
	public static double collection0LineupToPickupSpeed = 0.5;
	public static double collection0PickupToLaunchSpeed = 1;
	public static double launchToCollection1LineupSpeed = 1;
	public static double collection1LineupToPickupSpeed = 0.7;
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
	public static int linger0TimeMs = 600;
	public static int linger1TimeMs = 500;

	// Maximum time in each launch state before moving on (prevents infinite loops)
	// 9 attempts (3 slots × 3 attempts) at ~1 second each = ~9 seconds, plus buffer
	public static long maxLaunchStateTimeMs = 12000;
}
