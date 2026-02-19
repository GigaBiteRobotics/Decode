package org.firstinspires.ftc.teamcode.drive.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class BlueCloseAutoConstants12Ball {
	public static Pose startPose = new Pose(22.85, 124.85, -0.71);
	public static Pose launchPose = new Pose(55, 90, 1.5708);
	public static Pose gatePickupPose = new Pose(9, 66, -3.53); // Position to pickup from gate
	public static Pose set1LineupPose = new Pose(36, 60, Math.PI);
	public static Pose set1PickupPose = new Pose(11.5, 60, Math.PI);
	public static Pose finalPose = new Pose(20, 90, Math.PI);

	public static double elevationPos = 0.12;
	public static int targetRPM = 3300;
	public static double azimuthPos = 0.0;

	// Intake speeds (0.0 to 1.0)
	public static double intakeInSpeed = 1;

	// Robot drive speeds for each path (0.0 to 1.0)
	public static double defaultSpeed = 1;
	public static double pickupSpeed = 0.8;

	// Delay between shots in milliseconds (time between each pit fire)
	public static int shotDelayMs = 200;

	// Delay for each pit (pit 0, pit 1, pit 2) - settable individually
	public static int pit0DelayMs = 200;
	public static int pit1DelayMs = 200;
	public static int pit2DelayMs = 200;

	// Delay after launching before moving to next position in milliseconds
	public static int postLaunchDelayMs = 500;

	// Time to wait for ball collection at each position in milliseconds
	public static int intakeCollectTimeMs = 1800;

	// Time to linger at each ball position before moving (milliseconds)
	public static int lingerTimeMs = 400;

	// Maximum time in each launch state before moving on (prevents infinite loops)
	public static long maxLaunchStateTimeMs = 12000;
}
