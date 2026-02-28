package org.firstinspires.ftc.teamcode.drive.auto;

import org.firstinspires.ftc.teamcode.util.DashboardTelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.drive.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.drive.AutoToTeleDataTransferer;
import org.firstinspires.ftc.teamcode.drive.CustomPIDFController;
import org.firstinspires.ftc.teamcode.drive.CustomThreads;
import org.firstinspires.ftc.teamcode.drive.MDOConstants;
import org.firstinspires.ftc.teamcode.drive.HubInitializer;
import org.firstinspires.ftc.teamcode.drive.CustomAxonServoController;
import org.firstinspires.ftc.teamcode.drive.CustomMotorController;
import org.firstinspires.ftc.teamcode.drive.CustomMotor;
import org.firstinspires.ftc.teamcode.drive.CustomTelemetry;
import org.firstinspires.ftc.teamcode.drive.CustomSorterController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Red Close Auto", group = "Auto")
public class RedCloseAutoOpmode extends OpMode {

	// ===== STATE MACHINE =====
	protected enum AutoState {
		IDLE,
		SPIN_UP_LAUNCHER,
		DRIVE_TO_CAMERA_LOOK,
		CAMERA_LOOK,
		DRIVE_TO_LAUNCH,
		LAUNCH_BALLS,
		DRIVE_TO_BALL_0_LINEUP,
		DRIVE_TO_BALL_0_PICKUP,
		COLLECT_BALL_0,
		DRIVE_TO_LAUNCH_FROM_BALL_0,
		LAUNCH_BALLS_2,
		DRIVE_TO_BALL_1_LINEUP,
		DRIVE_TO_BALL_1_PICKUP,
		COLLECT_BALL_1,
		DRIVE_TO_LAUNCH_FROM_BALL_1,
		LAUNCH_BALLS_3,
		DRIVE_TO_FINAL,
		FINISHED
	}
	protected AutoState currentState = AutoState.IDLE;
	protected AutoState previousState = AutoState.IDLE;

	// ===== ROBOT COMPONENTS =====
	protected Follower follower;
	protected CustomMotorController launcherMotors;
	protected CustomMotorController intakeMotor;
	protected CustomSorterController sorterController;
	protected CustomAxonServoController azimuthServo;
	protected CustomAxonServoController elevationServo;
	protected AprilTagLocalizer aprilTagLocalizer;

	// ===== APRILTAG TRACKING =====
	protected List<Integer> detectedAprilTags = new ArrayList<>(); // List of all AprilTag IDs seen
	protected List<CustomSorterController.CustomColor> launchOrder = new ArrayList<>(); // Order to launch balls
	protected int launchOrderIndex = 0; // Current index in launch order
	protected boolean[] pitLaunched = new boolean[3]; // Track which pits have been launched this cycle
	protected CustomThreads customThreads;

	// ===== LAUNCHER CONTROL =====
	protected boolean launcherSpinning = false;
	protected int launcherRPM = RedCloseAutoConstants.targetRPM;

	// ===== INTAKE CONTROL =====
	protected enum IntakeState {
		IN,
		OUT,
		STOP
	}
	protected IntakeState intakeRunningState = IntakeState.STOP;

	// ===== TIMERS =====
	protected ElapsedTime runtime = new ElapsedTime();
	protected ElapsedTime stateTimer = new ElapsedTime();
	protected ElapsedTime launchTimer = new ElapsedTime();
	protected ElapsedTime intakeOffTimer = new ElapsedTime();
	protected boolean intakeOffTimerActive = false;
	protected int currentIntakePosition = 0; // Tracks which intake position (0 or 1) for intake collect time

	// ===== TELEMETRY =====
	protected CustomTelemetry telemetryC;
	protected static DashboardTelemetryManager telemetryM;

	// ===== PATHS =====
	protected PathChain pathToCameraLook; // From start to camera look (first thing)
	protected PathChain pathFromCameraLookToLaunch; // From camera look to launch
	protected PathChain pathToLaunch; // From start to launch (when camera look is skipped)
	protected PathChain pathToBall0Lineup;
	protected PathChain pathToBall0Pickup;
	protected PathChain pathFromBall0ToLaunch;
	protected PathChain pathToBall1Lineup;
	protected PathChain pathToBall1Pickup;
	protected PathChain pathFromBall1ToLaunch;
	protected PathChain pathToFinal;

	// ===== LAUNCH TRACKING =====
	protected int ballsLaunched = 0;
	protected static final int BALLS_TO_LAUNCH = 3;
	protected int currentLaunchSlot = 0; // Current slot being launched
	protected CustomSorterController.CustomColor lastLaunchedColor = null; // Track last color for delay

	// ===== OPTIONS =====
	protected boolean enableCameraLook = true; // Toggle camera look state

	@Override
	public void init() {
		// Initialize telemetry
		telemetryM = DashboardTelemetryManager.create();
		telemetryC = new CustomTelemetry(telemetry, telemetryM);
		telemetryC.addData("Status", "Initializing...");
		telemetryC.update();

		// Initialize follower with Pedro Pathing
		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(RedCloseAutoConstants.startPose);

		// Initialize hub bulk caching
		HubInitializer.initBulkCaching(hardwareMap);

		// Initialize sorter controller
		sorterController = new CustomSorterController(hardwareMap);

		// Initialize launcher motors (same as MainDriveOpmode)
		launcherMotors = new CustomMotorController(
				hardwareMap,
				new String[]{"launcher0", "launcher1"},
				new boolean[]{true, false}, // motor reverse map: launcher0 is reversed
				new boolean[]{true, false}, // encoder reverse map: launcher0 encoder is reversed to match motor direction
				new boolean[]{true, false}, // encoder enable map: only launcher0 has encoder
				32, // ticks per rev - calibrated for actual motor encoder (28 * 6700/6000)
				new CustomPIDFController(0, 0, 0, 0)
		);

		// Initialize intake motor (same as MainDriveOpmode)
		intakeMotor = new CustomMotorController(
			hardwareMap,
			new String[]{"intake"},
			new boolean[]{true},
			false,
			28.0,
			new CustomPIDFController(0, 0, 0, 0)
		);

		// Initialize azimuth servo (with PID for continuous rotation, but fixed position - no auto-aim)
		azimuthServo = new CustomAxonServoController(
			hardwareMap,
			new String[]{"azimuthServo0", "azimuthServo1"},
			new boolean[]{true, true}, // both reversed (same as MainDriveOpmode)
			true, // use analog position sensor for PID
			MDOConstants.AzimuthPIDFConstants, // PID constants from MDOConstants
			"azimuthPosition" // analog position sensor name
		);

		// Initialize elevation servo (simple mode, no PID - same as MainDriveOpmode)
		elevationServo = new CustomAxonServoController(
			hardwareMap,
			new String[]{"elevationServo"},
			new boolean[]{false},
			false, // no analog position sensor
			new double[]{0, 0, 0},
			null
		);

		// Initialize AprilTag localizer for camera vision
		aprilTagLocalizer = new AprilTagLocalizer();
		aprilTagLocalizer.cameraPosition = new Position(DistanceUnit.INCH,
			MDOConstants.CameraOffset[0], MDOConstants.CameraOffset[1], MDOConstants.CameraOffset[2], 0);
		aprilTagLocalizer.initAprilTag(hardwareMap, "Webcam 1");

		// Initialize custom threads
		customThreads = new CustomThreads(follower);
		customThreads.setLauncherMotors(launcherMotors);
		customThreads.setSorterController(sorterController);
		customThreads.setAzimuthServo(azimuthServo);

		// Build paths from constants
		buildPaths();

		telemetryC.addData("Status", "Initialized");
		telemetryC.addData("Start Pose", String.format("(%.1f, %.1f, %.1f°)",
			RedCloseAutoConstants.startPose.getX(),
			RedCloseAutoConstants.startPose.getY(),
			Math.toDegrees(RedCloseAutoConstants.startPose.getHeading())));
		telemetryC.update();
	}

	protected void buildPaths() {
		// Path from start to camera look position (first thing, when enabled)
		pathToCameraLook = follower.pathBuilder()
			.addPath(new BezierLine(
				RedCloseAutoConstants.startPose,
				RedCloseAutoConstants.cameraLookPose
			))
			.setLinearHeadingInterpolation(
				RedCloseAutoConstants.startPose.getHeading(),
				RedCloseAutoConstants.cameraLookPose.getHeading()
			)
			.build();

		// Path from camera look to launch position
		pathFromCameraLookToLaunch = follower.pathBuilder()
			.addPath(new BezierLine(
				RedCloseAutoConstants.cameraLookPose,
				RedCloseAutoConstants.launchPose
			))
			.setLinearHeadingInterpolation(
				RedCloseAutoConstants.cameraLookPose.getHeading(),
				RedCloseAutoConstants.launchPose.getHeading()
			)
			.build();

		// Path from start to launch position (when camera look is skipped)
		pathToLaunch = follower.pathBuilder()
			.addPath(new BezierLine(
				RedCloseAutoConstants.startPose,
				RedCloseAutoConstants.launchPose
			))
			.setLinearHeadingInterpolation(
				RedCloseAutoConstants.startPose.getHeading(),
				RedCloseAutoConstants.launchPose.getHeading()
			)
			.build();

		// Path to ball 0 lineup (from launch)
		pathToBall0Lineup = follower.pathBuilder()
			.addPath(new BezierLine(
				RedCloseAutoConstants.launchPose,
				RedCloseAutoConstants.ballCollection0LineupPose
			))
			.setLinearHeadingInterpolation(
				RedCloseAutoConstants.launchPose.getHeading(),
				RedCloseAutoConstants.ballCollection0LineupPose.getHeading()
			)
			.build();

		// Path to ball 0 pickup
		pathToBall0Pickup = follower.pathBuilder()
			.addPath(new BezierLine(
				RedCloseAutoConstants.ballCollection0LineupPose,
				RedCloseAutoConstants.ballCollection0PickupPose
			))
			.setConstantHeadingInterpolation(RedCloseAutoConstants.ballCollection0PickupPose.getHeading())
			.build();

		// Path from ball 0 pickup back to launch
		pathFromBall0ToLaunch = follower.pathBuilder()
			.addPath(new BezierLine(
				RedCloseAutoConstants.ballCollection0PickupPose,
				RedCloseAutoConstants.launchPose
			))
			.setLinearHeadingInterpolation(
				RedCloseAutoConstants.ballCollection0PickupPose.getHeading(),
				RedCloseAutoConstants.launchPose.getHeading()
			)
			.build();

		// Path to ball 1 lineup (from launch)
		pathToBall1Lineup = follower.pathBuilder()
			.addPath(new BezierLine(
				RedCloseAutoConstants.launchPose,
				RedCloseAutoConstants.getBallCollection1LineupPose
			))
			.setLinearHeadingInterpolation(
				RedCloseAutoConstants.launchPose.getHeading(),
				RedCloseAutoConstants.getBallCollection1LineupPose.getHeading()
			)
			.build();

		// Path to ball 1 pickup
		pathToBall1Pickup = follower.pathBuilder()
			.addPath(new BezierLine(
				RedCloseAutoConstants.getBallCollection1LineupPose,
				RedCloseAutoConstants.getBallCollection1PickupPose
			))
			.setConstantHeadingInterpolation(RedCloseAutoConstants.getBallCollection1PickupPose.getHeading())
			.build();

		// Path from ball 1 pickup back to launch
		pathFromBall1ToLaunch = follower.pathBuilder()
			.addPath(new BezierCurve(
				RedCloseAutoConstants.getBallCollection1PickupPose,
				RedCloseAutoConstants.collection1PickupToLaunchIntermediary,
				RedCloseAutoConstants.launchPose
			))
			.setLinearHeadingInterpolation(
				RedCloseAutoConstants.getBallCollection1PickupPose.getHeading(),
				RedCloseAutoConstants.launchPose.getHeading()
			)
			.build();

		// Path to final position (from launch)
		pathToFinal = follower.pathBuilder()
			.addPath(new BezierLine(
				RedCloseAutoConstants.launchPose,
				RedCloseAutoConstants.finalPose
			))
			.setLinearHeadingInterpolation(
				RedCloseAutoConstants.launchPose.getHeading(),
				RedCloseAutoConstants.finalPose.getHeading()
			)
			.build();
	}

	@Override
	public void init_loop() {
		// Scan for AprilTags during init so we already have detection when auto starts
		scanForAprilTags();

		telemetryC.addData("Status", "Ready to start");
		telemetryC.addData("Target RPM", launcherRPM);

		// Camera detection info
		telemetryC.addData("--- CAMERA ---", "");
		telemetryC.addData("Detected Tags", getDetectedTagsString());
		telemetryC.addData("Launch Order", getLaunchOrderString());
		telemetryC.addData("Live Tags", getLiveTagsString());
		telemetryC.update();
	}

	/**
	 * Get currently visible AprilTags (live detection, not stored list)
	 */
	protected String getLiveTagsString() {
		if (aprilTagLocalizer == null || aprilTagLocalizer.aprilTag == null) {
			return "Camera not ready";
		}
		try {
			List<AprilTagDetection> currentDetections = aprilTagLocalizer.aprilTag.getDetections();
			if (currentDetections.isEmpty()) {
				return "No tags visible";
			}
			StringBuilder sb = new StringBuilder();
			for (int i = 0; i < currentDetections.size(); i++) {
				AprilTagDetection detection = currentDetections.get(i);
				if (detection != null) {
					if (i > 0) sb.append(", ");
					sb.append("ID:");
					sb.append(detection.id);
					if (detection.metadata != null) {
						sb.append(" (");
						sb.append(detection.metadata.name);
						sb.append(")");
					}
				}
			}
			return sb.toString();
		} catch (Exception e) {
			return "Error: " + e.getMessage();
		}
	}

	@Override
	public void start() {
		runtime.reset();
		stateTimer.reset();

		// Start background threads (same as MainDriveOpmode)
		customThreads.startLauncherPIDThread();
		customThreads.startSorterThread();

		// Set azimuth to fixed position once at start (stays here for entire auto)
		azimuthServo.setPosition(RedCloseAutoConstants.azimuthPos);

		// Begin state machine
		setState(AutoState.SPIN_UP_LAUNCHER);
	}

	@Override
	public void loop() {
		// Update follower
		follower.update();

		// Track state changes
		boolean stateChanged = (currentState != previousState);
		previousState = currentState;

		// Handle current state
		handleState(stateChanged);

		// Update launcher RPM (same pattern as MainDriveOpmode)
		launcherMotors.setRPM(launcherSpinning ? launcherRPM : 0);
		launcherMotors.setPIDFController(MDOConstants.LauncherPIDF);

		// Run azimuth PID loop to maintain position (position set once at start)
		azimuthServo.servoPidLoop();

		// Set elevation to fixed position from constants
		elevationServo.setPosition(RedCloseAutoConstants.elevationPos);

		// Check if intake should be turned off after timer expires (while driving)
		int intakeCollectTime = (currentIntakePosition == 0) ?
			RedCloseAutoConstants.intakeCollect0TimeMs : RedCloseAutoConstants.intakeCollect1TimeMs;
		if (intakeOffTimerActive && intakeOffTimer.milliseconds() > intakeCollectTime) {
			intakeRunningState = IntakeState.STOP;
			intakeOffTimerActive = false;
		}

		// Update intake motor based on state (only IN or STOP, never OUT)
		switch (intakeRunningState) {
			case IN:
				intakeMotor.setPower(RedCloseAutoConstants.intakeInSpeed);
				break;
			case OUT:
			case STOP:
			default:
				intakeMotor.setPower(0);
				break;
		}

		// Update sorter lifters
		sorterController.lifterUpdater();

		// Update telemetry
		updateTelemetry();
	}

	protected void handleState(boolean stateChanged) {
		switch (currentState) {
			case IDLE:
				// Should not normally be here after start()
				break;

			case SPIN_UP_LAUNCHER:
				if (stateChanged) {
					// Start spinning the launcher
					launcherSpinning = true;
					stateTimer.reset();
				}
				// Wait for launcher to spin up (200ms)
				if (stateTimer.milliseconds() > 200) {
					// Skip camera look if we already have detection from init
					if (!launchOrder.isEmpty()) {
						// Already detected AprilTag during init, go straight to launch
						setState(AutoState.DRIVE_TO_LAUNCH);
					} else if (enableCameraLook) {
						setState(AutoState.DRIVE_TO_CAMERA_LOOK);
					} else {
						setState(AutoState.DRIVE_TO_LAUNCH);
					}
				}
				break;

			case DRIVE_TO_CAMERA_LOOK:
				if (stateChanged) {
					follower.followPath(pathToCameraLook, RedCloseAutoConstants.startToLaunchSpeed, true);
				}
				// Wait for path to start (minimum 100ms) and complete
				if (!follower.isBusy() && stateTimer.milliseconds() > 100) {
					setState(AutoState.CAMERA_LOOK);
				}
				break;

			case CAMERA_LOOK:
				if (stateChanged) {
					// If we already have a launch order from init, skip camera look
					// Otherwise clear and wait for detection
					if (launchOrder.isEmpty()) {
						detectedAprilTags.clear();
						launchOrderIndex = 0;
					}
				}
				// Continuously scan for AprilTags while waiting
				scanForAprilTags();
				// Proceed immediately once we have a launch order (tag detected)
				// Or timeout after cameraLookTimeMs as fallback
				if (!launchOrder.isEmpty()) {
					setState(AutoState.DRIVE_TO_LAUNCH);
				}
				break;

			case DRIVE_TO_LAUNCH:
				if (stateChanged) {
					// Check if we came from camera look or skipped it
					if (previousState == AutoState.CAMERA_LOOK) {
						// Coming from camera look - follow path from camera look to launch
						follower.followPath(pathFromCameraLookToLaunch, RedCloseAutoConstants.launchToCameraLookSpeed, true);
					} else {
						// Coming from start (skipped camera look) - follow full path
						follower.followPath(pathToLaunch, RedCloseAutoConstants.startToLaunchSpeed, true);
					}
				}
				// Must wait at least 500ms before checking isBusy (give path time to start)
				// OR timeout after 3 seconds as fallback
				if (stateTimer.milliseconds() > 500) {
					if (!follower.isBusy() || stateTimer.milliseconds() > 3000) {
						setState(AutoState.LAUNCH_BALLS);
					}
				}
				break;

			case LAUNCH_BALLS:
				if (stateChanged) {
					ballsLaunched = 0;
					launchOrderIndex = 0;
					currentLaunchSlot = 0;
					lastLaunchedColor = null;
					pitLaunched[0] = false;
					pitLaunched[1] = false;
					pitLaunched[2] = false;
					launchTimer.reset();
					// Actively hold position at launch pose to prevent going limp
					follower.holdPoint(RedCloseAutoConstants.launchPose);
					// Force a sensor read BEFORE locking to ensure we know what colors are in each pit
					sorterController.updateSensors();
					// Lock lifters to prevent balls from moving between pits during this launch cycle
					sorterController.lockLiftersForLaunch(true);
				}
				// Calculate required delay - add extra delay if color is changing
				long requiredDelay = RedCloseAutoConstants.shotDelayMs;
				if (lastLaunchedColor != null && !launchOrder.isEmpty() && launchOrderIndex < launchOrder.size()) {
					CustomSorterController.CustomColor nextColor = launchOrder.get(launchOrderIndex);
					if (nextColor != lastLaunchedColor) {
						requiredDelay += RedCloseAutoConstants.colorChangeDelayMs;
					}
				}
				// Launch one ball at a time based on launchOrder from AprilTag
				if (launchTimer.milliseconds() > requiredDelay && ballsLaunched < 3) {
					CustomSorterController.CustomColor launchedColor = null;

					// Use launchOrder if available to determine which color to launch next
					if (!launchOrder.isEmpty() && launchOrderIndex < launchOrder.size()) {
						CustomSorterController.CustomColor targetColor = launchOrder.get(launchOrderIndex);
						launchedColor = launchNextBallSorted(targetColor);
					}

					// Fallback: if couldn't launch from order, force sequential
					if (launchedColor == null && currentLaunchSlot < 3) {
						sorterController.forceLaunchSlot(currentLaunchSlot);
						currentLaunchSlot++;
						launchedColor = CustomSorterController.CustomColor.NULL; // Unknown color for forced launch
					}

					// Increment counters when a ball is launched
					if (launchedColor != null) {
						ballsLaunched++;
						lastLaunchedColor = launchedColor;
						// Always increment launchOrderIndex to stay in sync with launch cycles
						// This ensures we move to the next target color regardless of how the ball was launched
						if (launchOrderIndex < launchOrder.size()) {
							launchOrderIndex++;
						}
						launchTimer.reset();
					}
				}
				// After initial 3 launches, check for misfired balls during post-launch delay
				if (ballsLaunched >= 3 && launchTimer.milliseconds() > RedCloseAutoConstants.shotDelayMs) {
					// Force fresh sensor read to accurately check for remaining balls
					sorterController.updateSensors();
					// Check if there are still balls in the pits (misfires)
					if (sorterController.getCachedBallCount() > 0) {
						// Try to launch any remaining ball
						if (sorterController.launchCached(CustomSorterController.CustomColor.NULL)) {
							launchTimer.reset(); // Reset timer to give time for this launch
						}
					}
				}
				// Transition after all 3 balls have been launched, post-launch delay, AND no balls remain (or timeout)
				if ((ballsLaunched >= 3 && launchTimer.milliseconds() > RedCloseAutoConstants.postLaunchDelayMs
					&& sorterController.getCachedBallCount() == 0)
					|| stateTimer.milliseconds() > RedCloseAutoConstants.maxLaunchStateTimeMs) {
					sorterController.lockLiftersForLaunch(false);
					setState(AutoState.DRIVE_TO_BALL_0_LINEUP);
				}
				break;

			case DRIVE_TO_BALL_0_LINEUP:
				if (stateChanged) {
					// Turn on intake early so it's running before reaching pickup
					intakeRunningState = IntakeState.IN;
					// Always coming from launch position
					follower.followPath(pathToBall0Lineup, RedCloseAutoConstants.cameraLookToCollection0LineupSpeed, true);
				}
				// Wait for path to start (minimum 100ms) and complete
				if (!follower.isBusy() && stateTimer.milliseconds() > 100) {
					setState(AutoState.DRIVE_TO_BALL_0_PICKUP);
				}
				break;

			case DRIVE_TO_BALL_0_PICKUP:
				if (stateChanged) {
					// Intake already running from lineup state
					// Slow down for ball pickup using maxPower parameter
					follower.followPath(pathToBall0Pickup, RedCloseAutoConstants.collection0LineupToPickupSpeed, true);
				}
				// Wait for path to start (minimum 100ms) and complete
				if (!follower.isBusy() && stateTimer.milliseconds() > 100) {
					// Transition to linger state
					setState(AutoState.COLLECT_BALL_0);
				}
				break;

			case COLLECT_BALL_0:
				// Linger at position with intake running for the configured time
				if (stateTimer.milliseconds() > RedCloseAutoConstants.linger0TimeMs) {
					// Start intake off timer for while driving
					intakeOffTimer.reset();
					intakeOffTimerActive = true;
					currentIntakePosition = 0;
					setState(AutoState.DRIVE_TO_LAUNCH_FROM_BALL_0);
				}
				break;

			case DRIVE_TO_LAUNCH_FROM_BALL_0:
				if (stateChanged) {
					// Spin up launcher (intake will be stopped by timer)
					launcherSpinning = true;
					follower.followPath(pathFromBall0ToLaunch, RedCloseAutoConstants.collection0PickupToLaunchSpeed, true);
				}
				// Wait for path to start (minimum 100ms) and complete
				if (!follower.isBusy() && stateTimer.milliseconds() > 100) {
					setState(AutoState.LAUNCH_BALLS_2);
				}
				break;

			case LAUNCH_BALLS_2:
				if (stateChanged) {
					ballsLaunched = 0;
					currentLaunchSlot = 0;
					lastLaunchedColor = null;
					launchOrderIndex = 0; // Reset to start of pattern for this launch cycle
					pitLaunched[0] = false;
					pitLaunched[1] = false;
					pitLaunched[2] = false;
					launchTimer.reset();
					// Actively hold position at launch pose to prevent going limp
					follower.holdPoint(RedCloseAutoConstants.launchPose);
					// Force a sensor read BEFORE locking to ensure we know what colors are in each pit
					sorterController.updateSensors();
					// Lock lifters to prevent balls from moving between pits during this launch cycle
					sorterController.lockLiftersForLaunch(true);
				}
				// Calculate required delay - add extra delay if color is changing
				long requiredDelay2 = RedCloseAutoConstants.shotDelayMs;
				if (lastLaunchedColor != null && !launchOrder.isEmpty() && launchOrderIndex < launchOrder.size()) {
					CustomSorterController.CustomColor nextColor = launchOrder.get(launchOrderIndex);
					if (nextColor != lastLaunchedColor) {
						requiredDelay2 += RedCloseAutoConstants.colorChangeDelayMs;
					}
				}
				// Launch one ball at a time based on launchOrder from AprilTag
				if (launchTimer.milliseconds() > requiredDelay2 && ballsLaunched < 3) {
					CustomSorterController.CustomColor launchedColor = null;

					// Use launchOrder if available to determine which color to launch next
					if (!launchOrder.isEmpty() && launchOrderIndex < launchOrder.size()) {
						CustomSorterController.CustomColor targetColor = launchOrder.get(launchOrderIndex);
						launchedColor = launchNextBallSorted(targetColor);
					}

					// Fallback: if couldn't launch from order, force sequential
					if (launchedColor == null && currentLaunchSlot < 3) {
						sorterController.forceLaunchSlot(currentLaunchSlot);
						currentLaunchSlot++;
						launchedColor = CustomSorterController.CustomColor.NULL;
					}

					// Increment counters when a ball is launched
					if (launchedColor != null) {
						ballsLaunched++;
						lastLaunchedColor = launchedColor;
						// Always increment launchOrderIndex to stay in sync with launch cycles
						// This ensures we move to the next target color regardless of how the ball was launched
						if (launchOrderIndex < launchOrder.size()) {
							launchOrderIndex++;
						}
						launchTimer.reset();
					}
				}
				// After initial 3 launches, check for misfired balls during post-launch delay
				if (ballsLaunched >= 3 && launchTimer.milliseconds() > RedCloseAutoConstants.shotDelayMs) {
					// Force fresh sensor read to accurately check for remaining balls
					sorterController.updateSensors();
					// Check if there are still balls in the pits (misfires)
					if (sorterController.getCachedBallCount() > 0) {
						// Try to launch any remaining ball
						if (sorterController.launchCached(CustomSorterController.CustomColor.NULL)) {
							launchTimer.reset(); // Reset timer to give time for this launch
						}
					}
				}
				// Transition after all 3 balls have been launched, post-launch delay, AND no balls remain (or timeout)
				if ((ballsLaunched >= 3 && launchTimer.milliseconds() > RedCloseAutoConstants.postLaunchDelayMs
					&& sorterController.getCachedBallCount() == 0)
					|| stateTimer.milliseconds() > RedCloseAutoConstants.maxLaunchStateTimeMs) {
					sorterController.lockLiftersForLaunch(false);
					setState(AutoState.DRIVE_TO_BALL_1_LINEUP);
				}
				break;

			case DRIVE_TO_BALL_1_LINEUP:
				if (stateChanged) {
					// Turn on intake early so it's running before reaching pickup
					intakeRunningState = IntakeState.IN;
					follower.followPath(pathToBall1Lineup, RedCloseAutoConstants.launchToCollection1LineupSpeed, true);
				}
				// Wait for path to start (minimum 100ms) and complete
				if (!follower.isBusy() && stateTimer.milliseconds() > 100) {
					setState(AutoState.DRIVE_TO_BALL_1_PICKUP);
				}
				break;

			case DRIVE_TO_BALL_1_PICKUP:
				if (stateChanged) {
					// Intake already running from lineup state
					// Slow down for ball pickup using maxPower parameter
					follower.followPath(pathToBall1Pickup, RedCloseAutoConstants.collection1LineupToPickupSpeed, true);
				}
				// Wait for path to start (minimum 100ms) and complete
				if (!follower.isBusy() && stateTimer.milliseconds() > 100) {
					// Transition to linger state
					setState(AutoState.COLLECT_BALL_1);
				}
				break;

			case COLLECT_BALL_1:
				// Linger at position with intake running for the configured time
				if (stateTimer.milliseconds() > RedCloseAutoConstants.linger1TimeMs) {
					// Start intake off timer for while driving
					intakeOffTimer.reset();
					intakeOffTimerActive = true;
					currentIntakePosition = 1;
					setState(AutoState.DRIVE_TO_LAUNCH_FROM_BALL_1);
				}
				break;

			case DRIVE_TO_LAUNCH_FROM_BALL_1:
				if (stateChanged) {
					// Spin up launcher (intake will be stopped by timer)
					launcherSpinning = true;
					follower.followPath(pathFromBall1ToLaunch, RedCloseAutoConstants.collection1PickupToFinalSpeed, true);
				}
				// Wait for path to start (minimum 100ms) and complete
				if (!follower.isBusy() && stateTimer.milliseconds() > 100) {
					setState(AutoState.LAUNCH_BALLS_3);
				}
				break;

			case LAUNCH_BALLS_3:
				if (stateChanged) {
					ballsLaunched = 0;
					currentLaunchSlot = 0;
					lastLaunchedColor = null;
					launchOrderIndex = 0; // Reset to re-use the same sorting pattern
					pitLaunched[0] = false;
					pitLaunched[1] = false;
					pitLaunched[2] = false;
					launchTimer.reset();
					// Actively hold position at launch pose to prevent going limp
					follower.holdPoint(RedCloseAutoConstants.launchPose);
					// Force a sensor read BEFORE locking to ensure we know what colors are in each pit
					sorterController.updateSensors();
					// Lock lifters to prevent balls from moving between pits
					sorterController.lockLiftersForLaunch(true);
				}
				// Calculate required delay - add extra delay if color is changing
				long requiredDelay3 = RedCloseAutoConstants.shotDelayMs;
				if (lastLaunchedColor != null && !launchOrder.isEmpty() && launchOrderIndex < launchOrder.size()) {
					CustomSorterController.CustomColor nextColor = launchOrder.get(launchOrderIndex);
					if (nextColor != lastLaunchedColor) {
						requiredDelay3 += RedCloseAutoConstants.colorChangeDelayMs;
					}
				}
				// Launch one ball at a time based on launchOrder from AprilTag
				if (launchTimer.milliseconds() > requiredDelay3 && ballsLaunched < 3) {
					CustomSorterController.CustomColor launchedColor = null;

					// Use launchOrder if available to determine which color to launch next
					if (!launchOrder.isEmpty() && launchOrderIndex < launchOrder.size()) {
						CustomSorterController.CustomColor targetColor = launchOrder.get(launchOrderIndex);
						launchedColor = launchNextBallSorted(targetColor);
					}

					// Fallback: if couldn't launch from order, force sequential
					if (launchedColor == null && currentLaunchSlot < 3) {
						sorterController.forceLaunchSlot(currentLaunchSlot);
						currentLaunchSlot++;
						launchedColor = CustomSorterController.CustomColor.NULL;
					}

					// Increment counters when a ball is launched
					if (launchedColor != null) {
						ballsLaunched++;
						lastLaunchedColor = launchedColor;
						// Always increment launchOrderIndex to stay in sync with launch cycles
						// This ensures we move to the next target color regardless of how the ball was launched
						if (launchOrderIndex < launchOrder.size()) {
							launchOrderIndex++;
						}
						launchTimer.reset();
					}
				}
				// After initial 3 launches, check for misfired balls during post-launch delay
				if (ballsLaunched >= 3 && launchTimer.milliseconds() > RedCloseAutoConstants.shotDelayMs) {
					// Force fresh sensor read to accurately check for remaining balls
					sorterController.updateSensors();
					// Check if there are still balls in the pits (misfires)
					if (sorterController.getCachedBallCount() > 0) {
						// Try to launch any remaining ball
						if (sorterController.launchCached(CustomSorterController.CustomColor.NULL)) {
							launchTimer.reset(); // Reset timer to give time for this launch
						}
					}
				}
				// Transition after all 3 balls have been launched, post-launch delay, AND no balls remain (or timeout)
				if ((ballsLaunched >= 3 && launchTimer.milliseconds() > RedCloseAutoConstants.postLaunchDelayMs
					&& sorterController.getCachedBallCount() == 0)
					|| stateTimer.milliseconds() > RedCloseAutoConstants.maxLaunchStateTimeMs) {
					sorterController.lockLiftersForLaunch(false);
					setState(AutoState.DRIVE_TO_FINAL);
				}
				break;

			case DRIVE_TO_FINAL:
				if (stateChanged) {
					follower.followPath(pathToFinal, RedCloseAutoConstants.collection1PickupToFinalSpeed, true);
				}
				// Wait for path to start (minimum 100ms) and complete
				if (!follower.isBusy() && stateTimer.milliseconds() > 100) {
					setState(AutoState.FINISHED);
				}
				break;

			case FINISHED:
				if (stateChanged) {
					// Stop intake only, keep launcher running
					intakeRunningState = IntakeState.STOP;
				}
				// Auto complete - do nothing
				break;
		}
	}

	// Toggle intake state
	protected void toggleIntake() {
		if (intakeRunningState == IntakeState.IN) {
			intakeRunningState = IntakeState.STOP;
		} else {
			intakeRunningState = IntakeState.IN;
		}
	}

	// Set intake state directly
	protected void setIntake(IntakeState state) {
		intakeRunningState = state;
	}

	/**
	 * Launch the next ball based on current launch order pattern.
	 * Uses pitLaunched[] array to track which pits have already been launched this cycle.
	 * Searches for target color first, then alternate color, then any remaining pit.
	 * @param targetColor The preferred color to launch (from launchOrder)
	 * @return The color that was launched, or null if no ball was launched
	 */
	protected CustomSorterController.CustomColor launchNextBallSorted(CustomSorterController.CustomColor targetColor) {
		// Priority order: slot 2 first, then 1, then 0
		int[] slotPriority = {2, 1, 0};

		CustomSorterController.CustomColor GREEN =
			CustomSorterController.CustomColor.GREEN;
		CustomSorterController.CustomColor PURPLE =
			CustomSorterController.CustomColor.PURPLE;

		// First, try to find the exact target color in a pit that hasn't been launched yet
		for (int pit : slotPriority) {
			if (!pitLaunched[pit] && sorterController.getCachedColor(pit) == targetColor) {
				if (sorterController.launchFromPit(pit)) {
					pitLaunched[pit] = true;
					return targetColor;
				}
			}
		}

		// Target color not available - try the alternate color
		CustomSorterController.CustomColor alternateColor =
			(targetColor == GREEN) ? PURPLE : GREEN;

		for (int pit : slotPriority) {
			if (!pitLaunched[pit] && sorterController.getCachedColor(pit) == alternateColor) {
				if (sorterController.launchFromPit(pit)) {
					pitLaunched[pit] = true;
					return alternateColor;
				}
			}
		}

		// Neither color found - try launching from any remaining pit (sensor might not have detected)
		for (int pit : slotPriority) {
			if (!pitLaunched[pit]) {
				if (sorterController.launchFromPit(pit)) {
					pitLaunched[pit] = true;
					return CustomSorterController.CustomColor.NULL;
				}
			}
		}

		return null;
	}

	/**
	 * Scan for AprilTags and update the detected list and launch order.
	 * Call this during CAMERA_LOOK state to determine ball shooting order.
	 * Only cares about tags 21, 22, 23 - ignores all other AprilTags on the field.
	 * Stops the camera processor immediately once a valid tag is detected.
	 *
	 * AprilTag ID mapping (pattern repeats for each launch cycle):
	 * - Tag 21 = GPP (Green, Purple, Purple)
	 * - Tag 22 = PGP (Purple, Green, Purple)
	 * - Tag 23 = PPG (Purple, Purple, Green)
	 */
	protected void scanForAprilTags() {
		// Skip if camera already disabled or launch order already set
		if (aprilTagLocalizer == null || aprilTagLocalizer.aprilTag == null || !launchOrder.isEmpty()) {
			return;
		}

		try {
			List<AprilTagDetection> currentDetections = aprilTagLocalizer.aprilTag.getDetections();
			for (AprilTagDetection detection : currentDetections) {
				if (detection != null) {
					int tagId = detection.id;

					// Only care about our specific target tags (21, 22, 23)
					// Ignore all other AprilTags on the field
					if (tagId != 21 && tagId != 22 && tagId != 23) {
						continue;
					}

					// Add to list if not already seen
					if (!detectedAprilTags.contains(tagId)) {
						detectedAprilTags.add(tagId);
					}

					// Set launch order based on tag ID (only if not already set)
					if (launchOrder.isEmpty()) {
						CustomSorterController.CustomColor GREEN =
							CustomSorterController.CustomColor.GREEN;
						CustomSorterController.CustomColor PURPLE =
							CustomSorterController.CustomColor.PURPLE;

						// Add pattern 3 times (for 3 launch cycles, 9 balls total)
						for (int cycle = 0; cycle < 3; cycle++) {
							if (tagId == 21) {
								// GPP - Green, Purple, Purple
								launchOrder.add(GREEN);
								launchOrder.add(PURPLE);
								launchOrder.add(PURPLE);
							} else if (tagId == 22) {
								// PGP - Purple, Green, Purple
								launchOrder.add(PURPLE);
								launchOrder.add(GREEN);
								launchOrder.add(PURPLE);
							} else if (tagId == 23) {
								// PPG - Purple, Purple, Green
								launchOrder.add(PURPLE);
								launchOrder.add(PURPLE);
								launchOrder.add(GREEN);
							}
						}

						// Stop the camera processor immediately to save resources
						// We have what we need, no point in continuing to process frames
						stopAprilTagCamera();
						return; // Exit early since we found our tag
					}
				}
			}
		} catch (Exception e) {
			// Ignore exceptions during AprilTag detection
		}
	}

	/**
	 * Stop the AprilTag camera processor to save CPU/power resources.
	 * Called automatically once a valid target tag (21, 22, or 23) is detected.
	 */
	protected void stopAprilTagCamera() {
		try {
			if (aprilTagLocalizer != null && aprilTagLocalizer.visionPortal != null && aprilTagLocalizer.aprilTag != null) {
				// Disable the AprilTag processor through the VisionPortal
				aprilTagLocalizer.visionPortal.setProcessorEnabled(aprilTagLocalizer.aprilTag, false);
			}
		} catch (Exception e) {
			// Ignore exceptions when stopping camera
		}
	}

	/**
	 * Get list of all detected AprilTag IDs as a string for telemetry
	 */
	protected String getDetectedTagsString() {
		if (detectedAprilTags.isEmpty()) {
			return "None";
		}
		StringBuilder sb = new StringBuilder();
		for (int i = 0; i < detectedAprilTags.size(); i++) {
			if (i > 0) sb.append(", ");
			sb.append(detectedAprilTags.get(i));
		}
		return sb.toString();
	}

	/**
	 * Get launch order as a string for telemetry (e.g., "G, P, P")
	 */
	protected String getLaunchOrderString() {
		if (launchOrder.isEmpty()) {
			return "None";
		}
		StringBuilder sb = new StringBuilder();
		for (int i = 0; i < launchOrder.size(); i++) {
			if (i > 0) sb.append(", ");
			CustomSorterController.CustomColor color = launchOrder.get(i);
			if (color == CustomSorterController.CustomColor.GREEN) {
				sb.append("G");
			} else if (color == CustomSorterController.CustomColor.PURPLE) {
				sb.append("P");
			} else {
				sb.append("?");
			}
		}
		return sb.toString();
	}

	protected void setState(AutoState newState) {
		previousState = currentState;
		currentState = newState;
		stateTimer.reset();
	}

	protected void updateTelemetry() {
		telemetryC.addData("State", currentState.toString());
		telemetryC.addData("Runtime", String.format("%.1f s", runtime.seconds()));
		telemetryC.addData("State Timer", String.format("%.0f ms", stateTimer.milliseconds()));
		telemetryC.addData("Follower Busy", follower.isBusy());
		telemetryC.addData("Launcher Spinning", launcherSpinning);
		telemetryC.addData("RPM Mode", launcherMotors.isRPMMode());
		telemetryC.addData("Target RPM", launcherRPM);
		telemetryC.addData("Current RPM", String.format("%.0f", launcherMotors.getAverageRPM()));
		telemetryC.addData("Launcher PID Output", String.format("%.3f", launcherMotors.getPIDOutput()));
		telemetryC.addData("Intake State", intakeRunningState.toString());
		telemetryC.addData("Balls Launched", ballsLaunched);
		telemetryC.addData("Ball Count", sorterController.getCachedBallCount());

		// AprilTag detection info
		telemetryC.addData("Detected Tags", getDetectedTagsString());
		telemetryC.addData("Launch Order", getLaunchOrderString());
		telemetryC.addData("Launch Index", launchOrderIndex + "/" + launchOrder.size());

		Pose pose = follower.getPose();
		telemetryC.addData("Pose", String.format("(%.1f, %.1f, %.1f°)",
			pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading())));

		telemetryC.update();
	}

	@Override
	public void stop() {
		// Stop launcher
		launcherSpinning = false;
		launcherMotors.setRPM(0);

		// Stop intake
		intakeRunningState = IntakeState.STOP;
		intakeMotor.setPower(0);

		// Stop threads
		customThreads.stopLauncherPIDThread();
		customThreads.stopSorterThread();

		// Transfer data to TeleOp
		AutoToTeleDataTransferer dataTransfer = AutoToTeleDataTransferer.getInstance();
		if (follower != null) {
			dataTransfer.setEndPose(follower.getPose());
			follower.breakFollowing();
		}
		dataTransfer.setAllianceColor("RED");
		dataTransfer.setAutoCompleted(currentState == AutoState.FINISHED);
		dataTransfer.setAutoRuntime(runtime.seconds());
	}
}
