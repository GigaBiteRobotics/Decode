package org.firstinspires.ftc.teamcode.drive.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class BlueCloseAutoConstants {
 	public static Pose startPose = new Pose(22.85, 124.85, -0.71);
	public static Pose launchPose = new Pose(55, 90, 1.31);
	public static Pose cameraLookPose = new Pose(60, 90, 1.31); // Different heading for camera view
	public static Pose ballCollection0LineupPose = new Pose(48, 84, Math.PI);
	public static Pose ballCollection0PickupPose = new Pose(16, 84, Math.PI);
	public static Pose getBallCollection1LineupPose = new Pose(53, 60, Math.PI);
	public static Pose getBallCollection1PickupPose = new Pose(18, 60, Math.PI);
	public static Pose finalPose = new Pose(18, 90, Math.PI);
	public static double elevationPos = 0.12;
	public static int targetRPM = 1925;
	public static double azimuthPos = -0.4;

	// Intake speeds (0.0 to 1.0)
	public static double intakeInSpeed = 1;

	// Robot drive speed for pickup paths (0.0 to 1.0)
	public static double pickupDriveSpeed = 1;

	// Delay between shots in milliseconds
	public static int shotDelayMs = 700;

	// Delay after launching before moving to next position in milliseconds
	public static int postLaunchDelayMs = 1000;

	// Time to wait for ball collection in milliseconds
	public static int intakeCollectTimeMs = 800;

	// Maximum time in each launch state before moving on (prevents infinite loops)
	// 9 attempts (3 slots × 3 attempts) at ~1 second each = ~9 seconds, plus buffer
	public static long maxLaunchStateTimeMs = 12000;
}
