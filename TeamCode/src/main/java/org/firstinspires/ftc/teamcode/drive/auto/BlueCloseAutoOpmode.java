package org.firstinspires.ftc.teamcode.drive.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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
import org.firstinspires.ftc.teamcode.drive.RobotCoreCustom;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Blue Close Auto", group = "Auto")
public class BlueCloseAutoOpmode extends OpMode {

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
	protected RobotCoreCustom robotCoreCustom;
	protected RobotCoreCustom.CustomMotorController launcherMotors;
	protected RobotCoreCustom.CustomMotorController intakeMotor;
	protected RobotCoreCustom.CustomSorterController sorterController;
	protected RobotCoreCustom.CustomAxonServoController azimuthServo;
	protected RobotCoreCustom.CustomAxonServoController elevationServo;
	protected AprilTagLocalizer aprilTagLocalizer;

	// ===== APRILTAG TRACKING =====
	protected List<Integer> detectedAprilTags = new ArrayList<>(); // List of all AprilTag IDs seen
	protected List<RobotCoreCustom.CustomSorterController.CustomColor> launchOrder = new ArrayList<>(); // Order to launch balls
	protected int launchOrderIndex = 0; // Current index in launch order
	protected CustomThreads customThreads;

	// ===== LAUNCHER CONTROL =====
	protected boolean launcherSpinning = false;
	protected int launcherRPM = BlueCloseAutoConstants.targetRPM;

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

	// ===== TELEMETRY =====
	protected RobotCoreCustom.CustomTelemetry telemetryC;
	protected static TelemetryManager telemetryM;

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

	// ===== OPTIONS =====
	protected boolean enableCameraLook = true; // Toggle camera look state

	@Override
	public void init() {
		// Initialize telemetry
		telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
		telemetryC = new RobotCoreCustom.CustomTelemetry(telemetry, telemetryM);
		telemetryC.addData("Status", "Initializing...");
		telemetryC.update();

		// Initialize follower with Pedro Pathing
		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(BlueCloseAutoConstants.startPose);

		// Initialize robot core
		robotCoreCustom = new RobotCoreCustom(hardwareMap, follower);

		// Initialize sorter controller
		sorterController = new RobotCoreCustom.CustomSorterController(hardwareMap);

		// Initialize launcher motors (same as MainDriveOpmode)
		launcherMotors = new RobotCoreCustom.CustomMotorController(
			hardwareMap,
			new String[]{"launcher0", "launcher1"},
			new boolean[]{true, false}, // launcher1 is reversed
			new boolean[]{true, false}, // encoder reverse map
			true, // has encoders
			28.0, // ticks per rev
			new CustomPIDFController(0, 0, 0, 0)
		);

		// Initialize intake motor (same as MainDriveOpmode)
		intakeMotor = new RobotCoreCustom.CustomMotorController(
			hardwareMap,
			new String[]{"intake"},
			new boolean[]{true},
			false,
			28.0,
			new CustomPIDFController(0, 0, 0, 0)
		);

		// Initialize azimuth servo (with PID for continuous rotation, but fixed position - no auto-aim)
		azimuthServo = new RobotCoreCustom.CustomAxonServoController(
			hardwareMap,
			new String[]{"azimuthServo0", "azimuthServo1"},
			new boolean[]{true, true}, // both reversed (same as MainDriveOpmode)
			true, // use analog position sensor for PID
			MDOConstants.AzimuthPIDFConstants, // PID constants from MDOConstants
			"azimuthPosition" // analog position sensor name
		);

		// Initialize elevation servo (simple mode, no PID - same as MainDriveOpmode)
		elevationServo = new RobotCoreCustom.CustomAxonServoController(
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
		customThreads = new CustomThreads(robotCoreCustom, follower);
		customThreads.setLauncherMotors(launcherMotors);
		customThreads.setSorterController(sorterController);
		customThreads.setAzimuthServo(azimuthServo);

		// Build paths from constants
		buildPaths();

		telemetryC.addData("Status", "Initialized");
		telemetryC.addData("Start Pose", String.format("(%.1f, %.1f, %.1f°)",
			BlueCloseAutoConstants.startPose.getX(),
			BlueCloseAutoConstants.startPose.getY(),
			Math.toDegrees(BlueCloseAutoConstants.startPose.getHeading())));
		telemetryC.update();
	}

	protected void buildPaths() {
		// Path from start to camera look position (first thing, when enabled)
		pathToCameraLook = follower.pathBuilder()
			.addPath(new BezierLine(
				BlueCloseAutoConstants.startPose,
				BlueCloseAutoConstants.cameraLookPose
			))
			.setLinearHeadingInterpolation(
				BlueCloseAutoConstants.startPose.getHeading(),
				BlueCloseAutoConstants.cameraLookPose.getHeading()
			)
			.build();

		// Path from camera look to launch position
		pathFromCameraLookToLaunch = follower.pathBuilder()
			.addPath(new BezierLine(
				BlueCloseAutoConstants.cameraLookPose,
				BlueCloseAutoConstants.launchPose
			))
			.setLinearHeadingInterpolation(
				BlueCloseAutoConstants.cameraLookPose.getHeading(),
				BlueCloseAutoConstants.launchPose.getHeading()
			)
			.build();

		// Path from start to launch position (when camera look is skipped)
		pathToLaunch = follower.pathBuilder()
			.addPath(new BezierLine(
				BlueCloseAutoConstants.startPose,
				BlueCloseAutoConstants.launchPose
			))
			.setLinearHeadingInterpolation(
				BlueCloseAutoConstants.startPose.getHeading(),
				BlueCloseAutoConstants.launchPose.getHeading()
			)
			.build();

		// Path to ball 0 lineup (from launch)
		pathToBall0Lineup = follower.pathBuilder()
			.addPath(new BezierLine(
				BlueCloseAutoConstants.launchPose,
				BlueCloseAutoConstants.ballCollection0LineupPose
			))
			.setLinearHeadingInterpolation(
				BlueCloseAutoConstants.launchPose.getHeading(),
				BlueCloseAutoConstants.ballCollection0LineupPose.getHeading()
			)
			.build();

		// Path to ball 0 pickup
		pathToBall0Pickup = follower.pathBuilder()
			.addPath(new BezierLine(
				BlueCloseAutoConstants.ballCollection0LineupPose,
				BlueCloseAutoConstants.ballCollection0PickupPose
			))
			.setConstantHeadingInterpolation(BlueCloseAutoConstants.ballCollection0PickupPose.getHeading())
			.build();

		// Path from ball 0 pickup back to launch
		pathFromBall0ToLaunch = follower.pathBuilder()
			.addPath(new BezierLine(
				BlueCloseAutoConstants.ballCollection0PickupPose,
				BlueCloseAutoConstants.launchPose
			))
			.setLinearHeadingInterpolation(
				BlueCloseAutoConstants.ballCollection0PickupPose.getHeading(),
				BlueCloseAutoConstants.launchPose.getHeading()
			)
			.build();

		// Path to ball 1 lineup (from launch)
		pathToBall1Lineup = follower.pathBuilder()
			.addPath(new BezierLine(
				BlueCloseAutoConstants.launchPose,
				BlueCloseAutoConstants.getBallCollection1LineupPose
			))
			.setLinearHeadingInterpolation(
				BlueCloseAutoConstants.launchPose.getHeading(),
				BlueCloseAutoConstants.getBallCollection1LineupPose.getHeading()
			)
			.build();

		// Path to ball 1 pickup
		pathToBall1Pickup = follower.pathBuilder()
			.addPath(new BezierLine(
				BlueCloseAutoConstants.getBallCollection1LineupPose,
				BlueCloseAutoConstants.getBallCollection1PickupPose
			))
			.setConstantHeadingInterpolation(BlueCloseAutoConstants.getBallCollection1PickupPose.getHeading())
			.build();

		// Path from ball 1 pickup back to launch
		pathFromBall1ToLaunch = follower.pathBuilder()
			.addPath(new BezierLine(
				BlueCloseAutoConstants.getBallCollection1PickupPose,
				BlueCloseAutoConstants.launchPose
			))
			.setLinearHeadingInterpolation(
				BlueCloseAutoConstants.getBallCollection1PickupPose.getHeading(),
				BlueCloseAutoConstants.launchPose.getHeading()
			)
			.build();

		// Path to final position (from launch)
		pathToFinal = follower.pathBuilder()
			.addPath(new BezierLine(
				BlueCloseAutoConstants.launchPose,
				BlueCloseAutoConstants.finalPose
			))
			.setLinearHeadingInterpolation(
				BlueCloseAutoConstants.launchPose.getHeading(),
				BlueCloseAutoConstants.finalPose.getHeading()
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
		azimuthServo.setPosition(BlueCloseAutoConstants.azimuthPos);

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
		elevationServo.setPosition(BlueCloseAutoConstants.elevationPos);

		// Update intake motor based on state (only IN or STOP, never OUT)
		switch (intakeRunningState) {
			case IN:
				intakeMotor.setPower(BlueCloseAutoConstants.intakeInSpeed);
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
					follower.followPath(pathToCameraLook);
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
						follower.followPath(pathFromCameraLookToLaunch, true);
					} else {
						// Coming from start (skipped camera look) - follow full path
						follower.followPath(pathToLaunch, true);
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
					launchTimer.reset();
					// Actively hold position at launch pose to prevent going limp
					follower.holdPoint(BlueCloseAutoConstants.launchPose);
					// Lock lifters to prevent balls from moving between pits
					sorterController.lockLiftersForLaunch(true);
				}
				// Launch one ball at a time based on launchOrder from AprilTag
				if (launchTimer.milliseconds() > BlueCloseAutoConstants.shotDelayMs && ballsLaunched < 3) {
					boolean launched = false;

					// Use launchOrder if available to determine which color to launch next
					if (!launchOrder.isEmpty() && launchOrderIndex < launchOrder.size()) {
						RobotCoreCustom.CustomSorterController.CustomColor targetColor = launchOrder.get(launchOrderIndex);
						launched = launchNextBallSorted(targetColor);
					}

					// Fallback: if couldn't launch from order, force sequential
					if (!launched && currentLaunchSlot < 3) {
						sorterController.forceLaunchSlot(currentLaunchSlot);
						currentLaunchSlot++;
						launched = true;
					}

					// Always increment counters when a ball is launched
					if (launched) {
						ballsLaunched++;
						if (launchOrderIndex < launchOrder.size()) {
							launchOrderIndex++;
						}
						launchTimer.reset();
					}
				}
				// Transition after all 3 balls have been launched and post-launch delay, or timeout
				if ((ballsLaunched >= 3 && launchTimer.milliseconds() > BlueCloseAutoConstants.postLaunchDelayMs)
					|| stateTimer.milliseconds() > BlueCloseAutoConstants.maxLaunchStateTimeMs) {
					sorterController.lockLiftersForLaunch(false);
					setState(AutoState.DRIVE_TO_BALL_0_LINEUP);
				}
				break;

			case DRIVE_TO_BALL_0_LINEUP:
				if (stateChanged) {
					// Turn on intake early so it's running before reaching pickup
					intakeRunningState = IntakeState.IN;
					// Always coming from launch position
					follower.followPath(pathToBall0Lineup);
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
					follower.followPath(pathToBall0Pickup, BlueCloseAutoConstants.pickupDriveSpeed, true);
				}
				// Wait for path to start (minimum 100ms) and complete
				if (!follower.isBusy() && stateTimer.milliseconds() > 100) {
					setState(AutoState.COLLECT_BALL_0);
				}
				break;

			case COLLECT_BALL_0:
				if (stateChanged) {
					stateTimer.reset();
				}
				// Wait for ball collection (intake keeps running from previous state)
				if (stateTimer.milliseconds() > BlueCloseAutoConstants.intakeCollectTimeMs) {
					intakeRunningState = IntakeState.STOP;
					setState(AutoState.DRIVE_TO_LAUNCH_FROM_BALL_0);
				}
				break;

			case DRIVE_TO_LAUNCH_FROM_BALL_0:
				if (stateChanged) {
					// Stop intake, spin up launcher
					intakeRunningState = IntakeState.STOP;
					launcherSpinning = true;
					follower.followPath(pathFromBall0ToLaunch, true);
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
					launchTimer.reset();
					// Actively hold position at launch pose to prevent going limp
					follower.holdPoint(BlueCloseAutoConstants.launchPose);
					// Lock lifters to prevent balls from moving between pits
					sorterController.lockLiftersForLaunch(true);
				}
				// Launch one ball at a time based on launchOrder from AprilTag
				if (launchTimer.milliseconds() > BlueCloseAutoConstants.shotDelayMs && ballsLaunched < 3) {
					boolean launched = false;

					// Use launchOrder if available to determine which color to launch next
					if (!launchOrder.isEmpty() && launchOrderIndex < launchOrder.size()) {
						RobotCoreCustom.CustomSorterController.CustomColor targetColor = launchOrder.get(launchOrderIndex);
						launched = launchNextBallSorted(targetColor);
					}

					// Fallback: if couldn't launch from order, force sequential
					if (!launched && currentLaunchSlot < 3) {
						sorterController.forceLaunchSlot(currentLaunchSlot);
						currentLaunchSlot++;
						launched = true;
					}

					// Always increment counters when a ball is launched
					if (launched) {
						ballsLaunched++;
						if (launchOrderIndex < launchOrder.size()) {
							launchOrderIndex++;
						}
						launchTimer.reset();
					}
				}
				// Transition after all 3 balls have been launched and post-launch delay, or timeout
				if ((ballsLaunched >= 3 && launchTimer.milliseconds() > BlueCloseAutoConstants.postLaunchDelayMs)
					|| stateTimer.milliseconds() > BlueCloseAutoConstants.maxLaunchStateTimeMs) {
					sorterController.lockLiftersForLaunch(false);
					setState(AutoState.DRIVE_TO_BALL_1_LINEUP);
				}
				break;

			case DRIVE_TO_BALL_1_LINEUP:
				if (stateChanged) {
					// Turn on intake early so it's running before reaching pickup
					intakeRunningState = IntakeState.IN;
					follower.followPath(pathToBall1Lineup);
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
					follower.followPath(pathToBall1Pickup, BlueCloseAutoConstants.pickupDriveSpeed, true);
				}
				// Wait for path to start (minimum 100ms) and complete
				if (!follower.isBusy() && stateTimer.milliseconds() > 100) {
					setState(AutoState.COLLECT_BALL_1);
				}
				break;

			case COLLECT_BALL_1:
				if (stateChanged) {
					stateTimer.reset();
				}
				// Wait for ball collection (intake keeps running from previous state)
				if (stateTimer.milliseconds() > BlueCloseAutoConstants.intakeCollectTimeMs) {
					intakeRunningState = IntakeState.STOP;
					setState(AutoState.DRIVE_TO_LAUNCH_FROM_BALL_1);
				}
				break;

			case DRIVE_TO_LAUNCH_FROM_BALL_1:
				if (stateChanged) {
					// Stop intake, spin up launcher
					intakeRunningState = IntakeState.STOP;
					launcherSpinning = true;
					follower.followPath(pathFromBall1ToLaunch, true);
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
					launchTimer.reset();
					// Actively hold position at launch pose to prevent going limp
					follower.holdPoint(BlueCloseAutoConstants.launchPose);
					// Lock lifters to prevent balls from moving between pits
					sorterController.lockLiftersForLaunch(true);
				}
				// Launch one ball at a time based on launchOrder from AprilTag
				if (launchTimer.milliseconds() > BlueCloseAutoConstants.shotDelayMs && ballsLaunched < 3) {
					boolean launched = false;

					// Use launchOrder if available to determine which color to launch next
					if (!launchOrder.isEmpty() && launchOrderIndex < launchOrder.size()) {
						RobotCoreCustom.CustomSorterController.CustomColor targetColor = launchOrder.get(launchOrderIndex);
						launched = launchNextBallSorted(targetColor);
					}

					// Fallback: if couldn't launch from order, force sequential
					if (!launched && currentLaunchSlot < 3) {
						sorterController.forceLaunchSlot(currentLaunchSlot);
						currentLaunchSlot++;
						launched = true;
					}

					// Always increment counters when a ball is launched
					if (launched) {
						ballsLaunched++;
						if (launchOrderIndex < launchOrder.size()) {
							launchOrderIndex++;
						}
						launchTimer.reset();
					}
				}
				// Transition after all 3 balls have been launched and post-launch delay, or timeout
				if ((ballsLaunched >= 3 && launchTimer.milliseconds() > BlueCloseAutoConstants.postLaunchDelayMs)
					|| stateTimer.milliseconds() > BlueCloseAutoConstants.maxLaunchStateTimeMs) {
					sorterController.lockLiftersForLaunch(false);
					setState(AutoState.DRIVE_TO_FINAL);
				}
				break;

			case DRIVE_TO_FINAL:
				if (stateChanged) {
					follower.followPath(pathToFinal);
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
	 * Searches through remaining colors in the launch order to find one that's available.
	 * If target color isn't available, tries remaining colors in priority order.
	 * @param targetColor The preferred color to launch (from launchOrder)
	 * @return true if any ball was successfully launched, false if no balls available
	 */
	protected boolean launchNextBallSorted(RobotCoreCustom.CustomSorterController.CustomColor targetColor) {
		// First, try to launch the exact target color
		if (sorterController.launchCachedStrict(targetColor)) {
			return true;
		}

		// Target color not available - try the OTHER color instead
		// This ensures we launch a ball even if the exact order can't be maintained
		RobotCoreCustom.CustomSorterController.CustomColor GREEN =
			RobotCoreCustom.CustomSorterController.CustomColor.GREEN;
		RobotCoreCustom.CustomSorterController.CustomColor PURPLE =
			RobotCoreCustom.CustomSorterController.CustomColor.PURPLE;

		RobotCoreCustom.CustomSorterController.CustomColor alternateColor =
			(targetColor == GREEN) ? PURPLE : GREEN;

		if (sorterController.launchCachedStrict(alternateColor)) {
			return true;
		}

		// Neither color detected - try launching ANY ball that might be there
		// This handles cases where color sensors fail to detect a ball
		return sorterController.launchCached(RobotCoreCustom.CustomSorterController.CustomColor.NULL);
	}

	/**
	 * Scan for AprilTags and update the detected list and launch order.
	 * Call this during CAMERA_LOOK state to determine ball shooting order.
	 *
	 * AprilTag ID mapping (pattern repeats for each launch cycle):
	 * - Tag 21 = GPP (Green, Purple, Purple)
	 * - Tag 22 = PGP (Purple, Green, Purple)
	 * - Tag 23 = PPG (Purple, Purple, Green)
	 */
	protected void scanForAprilTags() {
		if (aprilTagLocalizer == null || aprilTagLocalizer.aprilTag == null) {
			return;
		}

		try {
			List<AprilTagDetection> currentDetections = aprilTagLocalizer.aprilTag.getDetections();
			for (AprilTagDetection detection : currentDetections) {
				if (detection != null && detection.metadata != null) {
					int tagId = detection.id;

					// Add to list if not already seen
					if (!detectedAprilTags.contains(tagId)) {
						detectedAprilTags.add(tagId);
					}

					// Set launch order based on tag ID (only if not already set)
					if (launchOrder.isEmpty()) {
						RobotCoreCustom.CustomSorterController.CustomColor GREEN =
							RobotCoreCustom.CustomSorterController.CustomColor.GREEN;
						RobotCoreCustom.CustomSorterController.CustomColor PURPLE =
							RobotCoreCustom.CustomSorterController.CustomColor.PURPLE;

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
					}
				}
			}
		} catch (Exception e) {
			// Ignore exceptions during AprilTag detection
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
			RobotCoreCustom.CustomSorterController.CustomColor color = launchOrder.get(i);
			if (color == RobotCoreCustom.CustomSorterController.CustomColor.GREEN) {
				sb.append("G");
			} else if (color == RobotCoreCustom.CustomSorterController.CustomColor.PURPLE) {
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
		dataTransfer.setAllianceColor("BLUE");
		dataTransfer.setAutoCompleted(currentState == AutoState.FINISHED);
		dataTransfer.setAutoRuntime(runtime.seconds());
	}
}
