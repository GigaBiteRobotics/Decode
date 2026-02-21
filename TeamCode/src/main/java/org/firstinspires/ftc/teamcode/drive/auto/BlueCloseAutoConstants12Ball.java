package org.firstinspires.ftc.teamcode.drive.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class BlueCloseAutoConstants12Ball {
	public static Pose startPose = new Pose(22.85, 124.85, -0.71);
	public static Pose launchPose = new Pose(55, 90, 1.5708);
	public static Pose gatePickupPose = new Pose(9, 64, -3.53); // Position to pickup from gate
	public static Pose launchToGateBezierPoint = new Pose(55, 77, 0);
	public static Pose gateToLaunchBezierPoint = new Pose(55, 77, 0);
	public static Pose set1LineupPose = new Pose(40, 60, Math.PI);
	public static Pose set1PickupPose = new Pose(10, 60, Math.PI);
	public static Pose set1PkupToLaunchBezierPoint = new Pose(60, 80, 0);
	public static Pose set0LineupPose = new Pose(48, 83, Math.PI);
	public static Pose set0PickupPose = new Pose(18, 83, Math.PI);
	public static Pose finalPose = new Pose(20, 90, Math.PI);
	public static double elevationPos = 0.12;
	public static int targetRPM = 3300;
	public static double azimuthPos = -0.42;

	// Intake speeds (0.0 to 1.0)
	public static double intakeInSpeed = 1;

	// Robot drive speeds for each path (0.0 to 1.0)
	public static double defaultSpeed = 1;
	public static double pickupSpeed = 0.7;

	// Delay between shots in milliseconds (time between each pit fire)
	public static int shotDelayMs = 200;

	// Delay for each pit (pit 0, pit 1, pit 2) - settable individually
	public static int pit0DelayMs = 300;
	public static int pit1DelayMs = 300;
	public static int pit2DelayMs = 400;

	// Delay after launching before moving to next position in milliseconds
	public static int postLaunchDelayMs = 1000;

	// Time to wait for ball collection at each position in milliseconds
	public static int intakeCollectTimeMs = 2100;

	// Time to linger at each ball position before moving (milliseconds)
	public static int lingerTimeMs = 0;

	// Time to linger at gate pickup position before moving (milliseconds)
	public static int gateLingerTimeMs = 1200;

	// Gate shake parameters - robot oscillates back and forth to help collect balls
	public static Pose gateShakeOffset = new Pose(0, -2, 0.1); // Offset applied to both shake endpoints (x, y, heading)
	public static double gateShakeDistance = 3.0; // Distance to shake forward/backward (inches)
	public static int gateShakePeriodMs = 300; // Time per direction before switching (milliseconds)
	public static double gateShakeSpeed = 1; // Speed during shaking (0.0 to 1.0)
	public static int gateShakeMaxTimeMs = 1400; // Max time for shaking before moving on (milliseconds)

	// Maximum time in each launch state before moving on (prevents infinite loops)
	public static long maxLaunchStateTimeMs = 12000;

	// Maximum total auto time in seconds before emergency drive to final pose
	public static double autoTimeLimitSeconds = 28.0;
}
