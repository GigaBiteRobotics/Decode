package org.firstinspires.ftc.teamcode.drive.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.AutoToTeleDataTransferer;
import org.firstinspires.ftc.teamcode.drive.CustomPIDFController;
import org.firstinspires.ftc.teamcode.drive.CustomThreads;
import org.firstinspires.ftc.teamcode.drive.MDOConstants;
import org.firstinspires.ftc.teamcode.drive.RobotCoreCustom;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Far Auto", group = "Auto")
public class BlueFarAutoOpmode extends OpMode {

	// ===== STATE MACHINE =====
	protected enum AutoState {
		IDLE,
		SPIN_UP_LAUNCHER,
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
		DRIVE_TO_FINAL
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
	protected CustomThreads customThreads;

	// ===== LAUNCHER CONTROL =====
	protected boolean launcherSpinning = false;
	protected int launcherRPM = BlueFarAutoConstants.targetRPM;

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
	protected RobotCoreCustom.CustomTelemetry telemetryC;
	protected static TelemetryManager telemetryM;

	// ===== PATHS =====
	protected PathChain pathToLaunch;
	protected PathChain pathToBall0Lineup;
	protected PathChain pathToBall0Pickup;
	protected PathChain pathFromBall0ToLaunch;
	protected PathChain pathToBall1Lineup;
	protected PathChain pathToBall1Pickup;
	protected PathChain pathFromBall1ToLaunch;

	// ===== LAUNCH TRACKING =====
	protected int ballsLaunched = 0;
	protected int currentLaunchSlot = 0; // Current slot being launched

	@Override
	public void init() {
		// Initialize telemetry
		telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
		telemetryC = new RobotCoreCustom.CustomTelemetry(telemetry, telemetryM);
		telemetryC.addData("Status", "Initializing...");
		telemetryC.update();

		// Initialize follower with Pedro Pathing
		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(BlueFarAutoConstants.startPose);

		// Initialize robot core
		robotCoreCustom = new RobotCoreCustom(hardwareMap, follower);

		// Initialize sorter controller
		sorterController = new RobotCoreCustom.CustomSorterController(hardwareMap);

		// Initialize launcher motors (same as MainDriveOpmode)
		launcherMotors = new RobotCoreCustom.CustomMotorController(
				hardwareMap,
				new String[]{"launcher0", "launcher1"},
				new boolean[]{true, false}, // motor reverse map: launcher0 is reversed
				new boolean[]{true, false}, // encoder reverse map: launcher0 encoder is reversed to match motor direction
				new boolean[]{true, false}, // encoder enable map: only launcher0 has encoder
				32, // ticks per rev - calibrated for actual motor encoder (28 * 6700/6000)
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

		// Initialize custom threads
		customThreads = new CustomThreads(robotCoreCustom, follower);
		customThreads.setLauncherMotors(launcherMotors);
		customThreads.setSorterController(sorterController);
		customThreads.setAzimuthServo(azimuthServo);

		// Build paths from constants
		buildPaths();

		telemetryC.addData("Status", "Initialized");
		telemetryC.addData("Start Pose", String.format("(%.1f, %.1f, %.1f°)",
			BlueFarAutoConstants.startPose.getX(),
			BlueFarAutoConstants.startPose.getY(),
			Math.toDegrees(BlueFarAutoConstants.startPose.getHeading())));
		telemetryC.update();
	}

	protected void buildPaths() {
		// Path from start to launch position
		pathToLaunch = follower.pathBuilder()
			.addPath(new BezierLine(
				BlueFarAutoConstants.startPose,
				BlueFarAutoConstants.launchPose
			))
			.setLinearHeadingInterpolation(
				BlueFarAutoConstants.startPose.getHeading(),
				BlueFarAutoConstants.launchPose.getHeading()
			)
			.build();

		// Path to ball 0 lineup (from launch)
		pathToBall0Lineup = follower.pathBuilder()
			.addPath(new BezierLine(
				BlueFarAutoConstants.launchPose,
				BlueFarAutoConstants.ballCollection0LineupPose
			))
			.setLinearHeadingInterpolation(
				BlueFarAutoConstants.launchPose.getHeading(),
				BlueFarAutoConstants.ballCollection0LineupPose.getHeading()
			)
			.build();

		// Path to ball 0 pickup
		pathToBall0Pickup = follower.pathBuilder()
			.addPath(new BezierLine(
				BlueFarAutoConstants.ballCollection0LineupPose,
				BlueFarAutoConstants.ballCollection0PickupPose
			))
			.setConstantHeadingInterpolation(BlueFarAutoConstants.ballCollection0PickupPose.getHeading())
			.build();

		// Path from ball 0 pickup back to launch
		pathFromBall0ToLaunch = follower.pathBuilder()
			.addPath(new BezierLine(
				BlueFarAutoConstants.ballCollection0PickupPose,
				BlueFarAutoConstants.launchPose
			))
			.setLinearHeadingInterpolation(
				BlueFarAutoConstants.ballCollection0PickupPose.getHeading(),
				BlueFarAutoConstants.launchPose.getHeading()
			)
			.build();

		// Path to ball 1 lineup (from launch)
		pathToBall1Lineup = follower.pathBuilder()
			.addPath(new BezierLine(
				BlueFarAutoConstants.launchPose,
				BlueFarAutoConstants.ballCollection1LineupPose
			))
			.setLinearHeadingInterpolation(
				BlueFarAutoConstants.launchPose.getHeading(),
				BlueFarAutoConstants.ballCollection1LineupPose.getHeading()
			)
			.build();

		// Path to ball 1 pickup
		pathToBall1Pickup = follower.pathBuilder()
			.addPath(new BezierLine(
				BlueFarAutoConstants.ballCollection1LineupPose,
				BlueFarAutoConstants.ballCollection1PickupPose
			))
			.setConstantHeadingInterpolation(BlueFarAutoConstants.ballCollection1PickupPose.getHeading())
			.build();

		// Path from ball 1 pickup back to launch
		pathFromBall1ToLaunch = follower.pathBuilder()
			.addPath(new BezierLine(
				BlueFarAutoConstants.ballCollection1PickupPose,
				BlueFarAutoConstants.launchPose
			))
			.setLinearHeadingInterpolation(
				BlueFarAutoConstants.ballCollection1PickupPose.getHeading(),
				BlueFarAutoConstants.launchPose.getHeading()
			)
			.build();
	}

	@Override
	public void init_loop() {
		telemetryC.addData("Status", "Ready to start");
		telemetryC.addData("Target RPM", launcherRPM);
		telemetryC.update();
	}

	@Override
	public void start() {
		runtime.reset();
		stateTimer.reset();

		// Start background threads (same as MainDriveOpmode)
		customThreads.startLauncherPIDThread();
		customThreads.startSorterThread();

		// Set azimuth to fixed position once at start (stays here for entire auto)
		azimuthServo.setPosition(BlueFarAutoConstants.azimuthPos);

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

		// Check auto time limit - interrupt everything and drive to final pose
		if (runtime.seconds() >= BlueFarAutoConstants.autoTimeLimitSeconds
				&& currentState != AutoState.DRIVE_TO_FINAL && currentState != AutoState.IDLE) {
			// Stop launcher and intake
			launcherSpinning = false;
			intakeRunningState = IntakeState.STOP;
			intakeOffTimerActive = false;
			sorterController.lockLiftersForLaunch(false);

			// Drive to final pose using holdPoint (actively drives to and holds position)
			follower.holdPoint(BlueFarAutoConstants.finalPose);
			setState(AutoState.DRIVE_TO_FINAL);
			stateChanged = true;
		}

		// Handle current state
		handleState(stateChanged);

		// Update launcher RPM (same pattern as MainDriveOpmode)
		launcherMotors.setRPM(launcherSpinning ? launcherRPM : 0);
		launcherMotors.setPIDFController(MDOConstants.LauncherPIDF);

		// Run azimuth PID loop to maintain position (position set once at start)
		azimuthServo.servoPidLoop();

		// Set elevation to fixed position from constants
		elevationServo.setPosition(BlueFarAutoConstants.elevationPos);

		// Check if intake should be turned off after timer expires (while driving)
		int intakeCollectTime = (currentIntakePosition == 0) ?
			BlueFarAutoConstants.intakeCollect0TimeMs : BlueFarAutoConstants.intakeCollect1TimeMs;
		if (intakeOffTimerActive && intakeOffTimer.milliseconds() > intakeCollectTime) {
			intakeRunningState = IntakeState.STOP;
			intakeOffTimerActive = false;
		}

		// Update intake motor based on state (only IN or STOP, never OUT)
		switch (intakeRunningState) {
			case IN:
				intakeMotor.setPower(BlueFarAutoConstants.intakeInSpeed);
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
					setState(AutoState.DRIVE_TO_LAUNCH);
				}
				break;

			case DRIVE_TO_LAUNCH:
				if (stateChanged) {
					follower.followPath(pathToLaunch, BlueFarAutoConstants.startToLaunchSpeed, true);
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
					currentLaunchSlot = 0;
					launchTimer.reset();
					// Actively hold position at launch pose to prevent going limp
					follower.holdPoint(BlueFarAutoConstants.launchPose);
					// Lock lifters to prevent balls from moving between pits during this launch cycle
					sorterController.lockLiftersForLaunch(true);
				}
				// Skip empty pits immediately, only wait for RPM on pits with balls
				while (currentLaunchSlot < 3
						&& sorterController.getCachedColor(currentLaunchSlot) == RobotCoreCustom.CustomSorterController.CustomColor.NULL) {
					currentLaunchSlot++;
				}
				// Fire the next pit that has a ball - wait until RPM is at threshold
				if (currentLaunchSlot < 3
						&& launchTimer.milliseconds() > BlueFarAutoConstants.minShotDelayMs
						&& launcherMotors.getAverageRPM() >= launcherRPM * BlueFarAutoConstants.rpmThresholdPercent) {
					sorterController.forceLaunchSlot(currentLaunchSlot);
					ballsLaunched++;
					currentLaunchSlot++;
					launchTimer.reset();
				}
				// After all 3 pits checked, wait postLaunchDelayMs for balls to settle, then re-check
				if (currentLaunchSlot >= 3) {
					if (launchTimer.milliseconds() > BlueFarAutoConstants.postLaunchDelayMs) {
						// Check if new balls appeared (sorter moved them into pits)
						if (sorterController.getCachedBallCount() > 0) {
							// Re-fire all 3 pits again
							currentLaunchSlot = 0;
							launchTimer.reset();
						} else {
							// No balls left, move on
							sorterController.lockLiftersForLaunch(false);
							setState(AutoState.DRIVE_TO_BALL_0_LINEUP);
						}
					}
				}
				break;

			case DRIVE_TO_BALL_0_LINEUP:
				if (stateChanged) {
					// Turn on intake early so it's running before reaching pickup
					intakeRunningState = IntakeState.IN;
					// Always coming from launch position
					follower.followPath(pathToBall0Lineup, BlueFarAutoConstants.launchToCollection0LineupSpeed, true);
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
					follower.followPath(pathToBall0Pickup, BlueFarAutoConstants.collection0LineupToPickupSpeed, true);
				}
				// Wait for path to start (minimum 100ms) and complete
				if (!follower.isBusy() && stateTimer.milliseconds() > 100) {
					// Transition to linger state
					setState(AutoState.COLLECT_BALL_0);
				}
				break;

			case COLLECT_BALL_0:
				// Linger at position with intake running for the configured time
				if (stateTimer.milliseconds() > BlueFarAutoConstants.linger0TimeMs) {
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
					follower.followPath(pathFromBall0ToLaunch, BlueFarAutoConstants.collection0PickupToLaunchSpeed, true);
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
					follower.holdPoint(BlueFarAutoConstants.launchPose);
					// Lock lifters to prevent balls from moving between pits during this launch cycle
					sorterController.lockLiftersForLaunch(true);
				}
				// Skip empty pits immediately, only wait for RPM on pits with balls
				while (currentLaunchSlot < 3
						&& sorterController.getCachedColor(currentLaunchSlot) == RobotCoreCustom.CustomSorterController.CustomColor.NULL) {
					currentLaunchSlot++;
				}
				// Fire the next pit that has a ball - wait until RPM is at threshold
				if (currentLaunchSlot < 3
						&& launchTimer.milliseconds() > BlueFarAutoConstants.minShotDelayMs
						&& launcherMotors.getAverageRPM() >= launcherRPM * BlueFarAutoConstants.rpmThresholdPercent) {
					sorterController.forceLaunchSlot(currentLaunchSlot);
					ballsLaunched++;
					currentLaunchSlot++;
					launchTimer.reset();
				}
				// After all 3 pits checked, wait postLaunchDelayMs for balls to settle, then re-check
				if (currentLaunchSlot >= 3) {
					if (launchTimer.milliseconds() > BlueFarAutoConstants.postLaunchDelayMs) {
						// Check if new balls appeared (sorter moved them into pits)
						if (sorterController.getCachedBallCount() > 0) {
							// Re-fire all 3 pits again
							currentLaunchSlot = 0;
							launchTimer.reset();
						} else {
							// No balls left, move on
							sorterController.lockLiftersForLaunch(false);
							setState(AutoState.DRIVE_TO_BALL_1_LINEUP);
						}
					}
				}
				break;

			case DRIVE_TO_BALL_1_LINEUP:
				if (stateChanged) {
					// Turn on intake early so it's running before reaching pickup
					intakeRunningState = IntakeState.IN;
					follower.followPath(pathToBall1Lineup, BlueFarAutoConstants.launchToCollection1LineupSpeed, true);
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
					follower.followPath(pathToBall1Pickup, BlueFarAutoConstants.collection1LineupToPickupSpeed, true);
				}
				// Wait for path to start (minimum 100ms) and complete
				if (!follower.isBusy() && stateTimer.milliseconds() > 100) {
					// Transition to linger state
					setState(AutoState.COLLECT_BALL_1);
				}
				break;

			case COLLECT_BALL_1:
				// Linger at position with intake running for the configured time
				if (stateTimer.milliseconds() > BlueFarAutoConstants.linger1TimeMs) {
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
					follower.followPath(pathFromBall1ToLaunch, BlueFarAutoConstants.collection1PickupToFinalSpeed, true);
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
					follower.holdPoint(BlueFarAutoConstants.launchPose);
					// Lock lifters to prevent balls from moving between pits
					sorterController.lockLiftersForLaunch(true);
				}
				// Skip empty pits immediately, only wait for RPM on pits with balls
				while (currentLaunchSlot < 3
						&& sorterController.getCachedColor(currentLaunchSlot) == RobotCoreCustom.CustomSorterController.CustomColor.NULL) {
					currentLaunchSlot++;
				}
				// Fire the next pit that has a ball - wait until RPM is at threshold
				if (currentLaunchSlot < 3
						&& launchTimer.milliseconds() > BlueFarAutoConstants.minShotDelayMs
						&& launcherMotors.getAverageRPM() >= launcherRPM * BlueFarAutoConstants.rpmThresholdPercent) {
					sorterController.forceLaunchSlot(currentLaunchSlot);
					ballsLaunched++;
					currentLaunchSlot++;
					launchTimer.reset();
				}
				// After all 3 pits checked, wait postLaunchDelayMs for balls to settle, then re-check
				if (currentLaunchSlot >= 3) {
					if (launchTimer.milliseconds() > BlueFarAutoConstants.postLaunchDelayMs) {
						// Check if new balls appeared (sorter moved them into pits)
						if (sorterController.getCachedBallCount() > 0) {
							// Re-fire all 3 pits again
							currentLaunchSlot = 0;
							launchTimer.reset();
						} else {
							// No balls left, move on
							sorterController.lockLiftersForLaunch(false);
							setState(AutoState.DRIVE_TO_BALL_1_LINEUP);
						}
					}
				}
				break;

			case DRIVE_TO_FINAL:
				// holdPoint is continuously driving to and holding the final pose
				// Nothing else to do - just stay here until opmode ends
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
		dataTransfer.setAutoCompleted(true); // Auto loops until stopped
		dataTransfer.setAutoRuntime(runtime.seconds());
	}
}

