package org.firstinspires.ftc.teamcode.drive.auto;

import org.firstinspires.ftc.teamcode.util.DashboardTelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name = "Blue Close Auto 12 Ball", group = "Auto")
public class BlueCloseAutoOpmode12Ball extends OpMode {

	// ===== STATE MACHINE =====
	// Sequence (4 launches, 12 balls):
	// 1. Launch preloaded
	// 2. Pickup Set 1 -> Launch
	// 3. Pickup from gate -> Launch
	// 4. Pickup Set 0 -> Launch
	// 5. Final pose
	protected enum AutoState {
		IDLE,
		SPIN_UP_LAUNCHER,
		// Launch 1: Preloaded
		DRIVE_TO_LAUNCH,
		LAUNCH_1,
		// Launch 2: Pickup Set 1 -> Launch
		DRIVE_TO_SET1_LINEUP,
		DRIVE_TO_SET1_PICKUP,
		COLLECT_SET1,
		DRIVE_TO_LAUNCH_2,
		LAUNCH_2,
		// Launch 3: Pickup from gate -> Launch
		DRIVE_TO_GATE_PICKUP,
		SHAKE_AT_GATE,
		COLLECT_GATE,
		DRIVE_TO_LAUNCH_3,
		LAUNCH_3,
		// Launch 4: Pickup Set 0 -> Launch
		DRIVE_TO_SET0_LINEUP,
		DRIVE_TO_SET0_PICKUP,
		COLLECT_SET0,
		DRIVE_TO_LAUNCH_4,
		LAUNCH_4,
		// Final
		DRIVE_TO_FINAL,
		EMERGENCY_DRIVE_TO_FINAL,
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
	protected CustomThreads customThreads;

	// ===== LAUNCHER CONTROL =====
	protected boolean launcherSpinning = false;
	protected int launcherRPM = BlueCloseAutoConstants12Ball.targetRPM;

	// ===== INTAKE CONTROL =====
	protected enum IntakeState { IN, OUT, STOP }
	protected IntakeState intakeRunningState = IntakeState.STOP;

	// ===== TIMERS =====
	protected ElapsedTime runtime = new ElapsedTime();
	protected ElapsedTime stateTimer = new ElapsedTime();
	protected ElapsedTime launchTimer = new ElapsedTime();
	protected ElapsedTime intakeOffTimer = new ElapsedTime();
	protected boolean intakeOffTimerActive = false;

	// ===== TELEMETRY =====
	protected CustomTelemetry telemetryC;
	protected static DashboardTelemetryManager telemetryM;

	// ===== PATHS =====
	protected PathChain pathToLaunch;
	protected PathChain pathToSet1Lineup, pathToSet1Pickup, pathFromSet1ToLaunch;
	protected PathChain pathToGatePickup, pathFromGateToLaunch;
	protected PathChain pathGateShakeForward, pathGateShakeBackward;
	protected PathChain pathToSet0Lineup, pathToSet0Pickup, pathFromSet0ToLaunch;
	protected PathChain pathToFinal;

	// ===== LAUNCH TRACKING =====
	protected int ballsLaunched = 0;
	protected int currentLaunchSlot = 0;
	protected boolean initialLaunchDone = false;
	protected ElapsedTime recheckTimer = new ElapsedTime();

	// ===== GATE SHAKE TRACKING =====
	protected boolean gateShakeForward = true;
	protected int gateShakesCompleted = 0;
	protected ElapsedTime gateShakeTimer = new ElapsedTime();

	@Override
	public void init() {
		// Initialize telemetry
		telemetryM = DashboardTelemetryManager.create();
		telemetryC = new CustomTelemetry(telemetry, telemetryM);
		telemetryC.addData("Status", "Initializing...");
		telemetryC.update();

		// Initialize follower with Pedro Pathing
		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(BlueCloseAutoConstants12Ball.startPose);

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


		// Initialize custom threads
		customThreads = new CustomThreads(follower);
		customThreads.setLauncherMotors(launcherMotors);
		customThreads.setSorterController(sorterController);
		customThreads.setAzimuthServo(azimuthServo);

		// Build paths from constants
		buildPaths();

		telemetryC.addData("Status", "Initialized");
		telemetryC.addData("Start Pose", String.format("(%.1f, %.1f, %.1f°)",
			BlueCloseAutoConstants12Ball.startPose.getX(),
			BlueCloseAutoConstants12Ball.startPose.getY(),
			Math.toDegrees(BlueCloseAutoConstants12Ball.startPose.getHeading())));
		telemetryC.update();
	}

	protected void buildPaths() {
		// Start to launch
		pathToLaunch = follower.pathBuilder()
			.addPath(new BezierLine(BlueCloseAutoConstants12Ball.startPose, BlueCloseAutoConstants12Ball.launchPose))
			.setLinearHeadingInterpolation(BlueCloseAutoConstants12Ball.startPose.getHeading(), BlueCloseAutoConstants12Ball.launchPose.getHeading())
			.build();

		// Set 1 paths
		pathToSet1Lineup = follower.pathBuilder()
			.addPath(new BezierLine(BlueCloseAutoConstants12Ball.launchPose, BlueCloseAutoConstants12Ball.set1LineupPose))
			.setLinearHeadingInterpolation(BlueCloseAutoConstants12Ball.launchPose.getHeading(), BlueCloseAutoConstants12Ball.set1LineupPose.getHeading())
			.build();

		pathToSet1Pickup = follower.pathBuilder()
			.addPath(new BezierLine(BlueCloseAutoConstants12Ball.set1LineupPose, BlueCloseAutoConstants12Ball.set1PickupPose))
			.setLinearHeadingInterpolation(BlueCloseAutoConstants12Ball.set1LineupPose.getHeading(), BlueCloseAutoConstants12Ball.set1PickupPose.getHeading())
			.build();

		pathFromSet1ToLaunch = follower.pathBuilder()
			.addPath(new BezierCurve(BlueCloseAutoConstants12Ball.set1PickupPose, BlueCloseAutoConstants12Ball.set1PkupToLaunchBezierPoint, BlueCloseAutoConstants12Ball.launchPose))
			.setLinearHeadingInterpolation(BlueCloseAutoConstants12Ball.set1PickupPose.getHeading(), BlueCloseAutoConstants12Ball.launchPose.getHeading())
			.build();

		// Gate paths
		pathToGatePickup = follower.pathBuilder()
			.addPath(new BezierCurve(BlueCloseAutoConstants12Ball.launchPose,BlueCloseAutoConstants12Ball.launchToGateBezierPoint, BlueCloseAutoConstants12Ball.gatePickupPose))
			.setLinearHeadingInterpolation(BlueCloseAutoConstants12Ball.launchPose.getHeading(), BlueCloseAutoConstants12Ball.gatePickupPose.getHeading())
			.build();

		pathFromGateToLaunch = follower.pathBuilder()
			.addPath(new BezierCurve(BlueCloseAutoConstants12Ball.gatePickupPose,BlueCloseAutoConstants12Ball.gateToLaunchBezierPoint, BlueCloseAutoConstants12Ball.launchPose))
			.setLinearHeadingInterpolation(BlueCloseAutoConstants12Ball.gatePickupPose.getHeading(), BlueCloseAutoConstants12Ball.launchPose.getHeading())
			.build();

		// Gate shake paths - oscillate forward/backward from gate pickup pose along its heading
		double shakeHeading = BlueCloseAutoConstants12Ball.gatePickupPose.getHeading() + BlueCloseAutoConstants12Ball.gateShakeOffset.getHeading();
		double shakeDist = BlueCloseAutoConstants12Ball.gateShakeDistance;
		Pose gateShakeBasePose = new Pose(
			BlueCloseAutoConstants12Ball.gatePickupPose.getX() + BlueCloseAutoConstants12Ball.gateShakeOffset.getX(),
			BlueCloseAutoConstants12Ball.gatePickupPose.getY() + BlueCloseAutoConstants12Ball.gateShakeOffset.getY(),
			shakeHeading
		);
		Pose gateShakeForwardPose = new Pose(
			gateShakeBasePose.getX() + shakeDist * Math.cos(shakeHeading),
			gateShakeBasePose.getY() + shakeDist * Math.sin(shakeHeading),
			shakeHeading
		);
		pathGateShakeForward = follower.pathBuilder()
			.addPath(new BezierLine(gateShakeBasePose, gateShakeForwardPose))
			.setConstantHeadingInterpolation(shakeHeading)
			.build();
		pathGateShakeBackward = follower.pathBuilder()
			.addPath(new BezierLine(gateShakeForwardPose, gateShakeBasePose))
			.setConstantHeadingInterpolation(shakeHeading)
			.build();

		// Set 0 paths
		pathToSet0Lineup = follower.pathBuilder()
			.addPath(new BezierLine(BlueCloseAutoConstants12Ball.launchPose, BlueCloseAutoConstants12Ball.set0LineupPose))
			.setLinearHeadingInterpolation(BlueCloseAutoConstants12Ball.launchPose.getHeading(), BlueCloseAutoConstants12Ball.set0LineupPose.getHeading())
			.build();

		pathToSet0Pickup = follower.pathBuilder()
			.addPath(new BezierLine(BlueCloseAutoConstants12Ball.set0LineupPose, BlueCloseAutoConstants12Ball.set0PickupPose))
			.setLinearHeadingInterpolation(BlueCloseAutoConstants12Ball.set0LineupPose.getHeading(), BlueCloseAutoConstants12Ball.set0PickupPose.getHeading())
			.build();

		pathFromSet0ToLaunch = follower.pathBuilder()
			.addPath(new BezierLine(BlueCloseAutoConstants12Ball.set0PickupPose, BlueCloseAutoConstants12Ball.launchPose))
			.setLinearHeadingInterpolation(BlueCloseAutoConstants12Ball.set0PickupPose.getHeading(), BlueCloseAutoConstants12Ball.launchPose.getHeading())
			.build();

		// Final
		pathToFinal = follower.pathBuilder()
			.addPath(new BezierLine(BlueCloseAutoConstants12Ball.launchPose, BlueCloseAutoConstants12Ball.finalPose))
			.setLinearHeadingInterpolation(BlueCloseAutoConstants12Ball.launchPose.getHeading(), BlueCloseAutoConstants12Ball.finalPose.getHeading())
			.build();
	}

	@Override
	public void init_loop() {
		telemetryC.addData("Status", "Ready to start");
		telemetryC.addData("Target RPM", launcherRPM);
		telemetryC.addData("Pit Delays", String.format("0:%dms, 1:%dms, 2:%dms",
			BlueCloseAutoConstants12Ball.pit0DelayMs,
			BlueCloseAutoConstants12Ball.pit1DelayMs,
			BlueCloseAutoConstants12Ball.pit2DelayMs));
		telemetryC.update();
	}

	@Override
	public void start() {
		runtime.reset();
		stateTimer.reset();
		customThreads.startLauncherPIDThread();
		customThreads.startSorterThread();
		azimuthServo.setPosition(BlueCloseAutoConstants12Ball.azimuthPos);
		setState(AutoState.SPIN_UP_LAUNCHER);
	}

	@Override
	public void loop() {
		follower.update();

		// Emergency timer: if auto time limit reached, stop everything and go to final pose
		if (runtime.seconds() >= BlueCloseAutoConstants12Ball.autoTimeLimitSeconds
				&& currentState != AutoState.EMERGENCY_DRIVE_TO_FINAL
				&& currentState != AutoState.DRIVE_TO_FINAL
				&& currentState != AutoState.FINISHED) {
			// Stop launcher and intake immediately
			launcherSpinning = false;
			intakeRunningState = IntakeState.STOP;
			intakeOffTimerActive = false;
			sorterController.lockLiftersForLaunch(false);

			// Build an emergency path from current position to final pose
			Pose currentPose = follower.getPose();
			PathChain emergencyPath = follower.pathBuilder()
				.addPath(new BezierLine(currentPose, BlueCloseAutoConstants12Ball.finalPose))
				.setLinearHeadingInterpolation(currentPose.getHeading(), BlueCloseAutoConstants12Ball.finalPose.getHeading())
				.build();
			follower.followPath(emergencyPath, BlueCloseAutoConstants12Ball.defaultSpeed, true);
			setState(AutoState.EMERGENCY_DRIVE_TO_FINAL);
		}

		boolean stateChanged = (currentState != previousState);
		previousState = currentState;

		handleState(stateChanged);

		launcherMotors.setRPM(launcherSpinning ? launcherRPM : 0);
		launcherMotors.setPIDFController(MDOConstants.LauncherPIDF);
		azimuthServo.servoPidLoop();
		elevationServo.setPosition(BlueCloseAutoConstants12Ball.elevationPos);

		if (intakeOffTimerActive && intakeOffTimer.milliseconds() > BlueCloseAutoConstants12Ball.intakeCollectTimeMs) {
			intakeRunningState = IntakeState.STOP;
			intakeOffTimerActive = false;
		}

		switch (intakeRunningState) {
			case IN: intakeMotor.setPower(BlueCloseAutoConstants12Ball.intakeInSpeed); break;
			default: intakeMotor.setPower(0); break;
		}

		sorterController.lifterUpdater();
		updateTelemetry();
	}

	protected void handleState(boolean stateChanged) {
		switch (currentState) {
			case IDLE:
				break;

			case SPIN_UP_LAUNCHER:
				if (stateChanged) {
					launcherSpinning = true;
				}
				if (stateTimer.milliseconds() > 200) {
					setState(AutoState.DRIVE_TO_LAUNCH);
				}
				break;

			// ===== LAUNCH 1: Preloaded =====
			case DRIVE_TO_LAUNCH:
				if (stateChanged) {
					follower.followPath(pathToLaunch, BlueCloseAutoConstants12Ball.defaultSpeed, true);
				}
				if (pathDone()) setState(AutoState.LAUNCH_1);
				break;

			case LAUNCH_1:
				handleLaunch(stateChanged, AutoState.DRIVE_TO_SET1_LINEUP);
				break;

			// ===== LAUNCH 2: Set 1 -> Launch =====
			case DRIVE_TO_SET1_LINEUP:
				if (stateChanged) {
					intakeRunningState = IntakeState.IN;
					follower.followPath(pathToSet1Lineup, BlueCloseAutoConstants12Ball.defaultSpeed, true);
				}
				if (pathDone()) setState(AutoState.DRIVE_TO_SET1_PICKUP);
				break;

			case DRIVE_TO_SET1_PICKUP:
				if (stateChanged) {
					follower.followPath(pathToSet1Pickup, BlueCloseAutoConstants12Ball.pickupSpeed, true);
				}
				if (pathDone()) setState(AutoState.COLLECT_SET1);
				break;

			case COLLECT_SET1:
				if (stateTimer.milliseconds() > BlueCloseAutoConstants12Ball.lingerTimeMs) {
					intakeOffTimer.reset();
					intakeOffTimerActive = true;
					setState(AutoState.DRIVE_TO_LAUNCH_2);
				}
				break;

			case DRIVE_TO_LAUNCH_2:
				if (stateChanged) {
					launcherSpinning = true;
					follower.followPath(pathFromSet1ToLaunch, BlueCloseAutoConstants12Ball.defaultSpeed, true);
				}
				if (pathDone()) setState(AutoState.LAUNCH_2);
				break;

			case LAUNCH_2:
				handleLaunch(stateChanged, AutoState.DRIVE_TO_GATE_PICKUP);
				break;

			// ===== LAUNCH 3: Gate Pickup -> Launch =====
			case DRIVE_TO_GATE_PICKUP:
				if (stateChanged) {
					intakeRunningState = IntakeState.IN;
					follower.followPath(pathToGatePickup, BlueCloseAutoConstants12Ball.defaultSpeed, true);
				}
				if (pathDone()) setState(AutoState.SHAKE_AT_GATE);
				break;

			case SHAKE_AT_GATE:
				if (stateChanged) {
					gateShakeForward = true;
					gateShakesCompleted = 0;
					gateShakeTimer.reset();
					follower.followPath(pathGateShakeForward, BlueCloseAutoConstants12Ball.gateShakeSpeed, true);
				}
				// Max time safety - move on regardless
				if (stateTimer.milliseconds() > BlueCloseAutoConstants12Ball.gateShakeMaxTimeMs) {
					setState(AutoState.COLLECT_GATE);
				}
				// Switch direction every period (keeps shaking until max time timer expires)
				else if (gateShakeTimer.milliseconds() > BlueCloseAutoConstants12Ball.gateShakePeriodMs) {
					gateShakeTimer.reset();
					if (gateShakeForward) {
						gateShakeForward = false;
						follower.followPath(pathGateShakeBackward, BlueCloseAutoConstants12Ball.gateShakeSpeed, true);
					} else {
						gateShakeForward = true;
						follower.followPath(pathGateShakeForward, BlueCloseAutoConstants12Ball.gateShakeSpeed, true);
					}
				}
				break;

			case COLLECT_GATE:
				if (stateTimer.milliseconds() > BlueCloseAutoConstants12Ball.gateLingerTimeMs) {
					intakeOffTimer.reset();
					intakeOffTimerActive = true;
					setState(AutoState.DRIVE_TO_LAUNCH_3);
				}
				break;

			case DRIVE_TO_LAUNCH_3:
				if (stateChanged) {
					launcherSpinning = true;
					follower.followPath(pathFromGateToLaunch, BlueCloseAutoConstants12Ball.defaultSpeed, true);
				}
				if (pathDone()) setState(AutoState.LAUNCH_3);
				break;

			case LAUNCH_3:
				handleLaunch(stateChanged, AutoState.DRIVE_TO_SET0_LINEUP);
				break;

			// ===== LAUNCH 4: Set 0 -> Launch =====
			case DRIVE_TO_SET0_LINEUP:
				if (stateChanged) {
					intakeRunningState = IntakeState.IN;
					follower.followPath(pathToSet0Lineup, BlueCloseAutoConstants12Ball.defaultSpeed, true);
				}
				if (pathDone()) setState(AutoState.DRIVE_TO_SET0_PICKUP);
				break;

			case DRIVE_TO_SET0_PICKUP:
				if (stateChanged) {
					follower.followPath(pathToSet0Pickup, BlueCloseAutoConstants12Ball.pickupSpeed, true);
				}
				if (pathDone()) setState(AutoState.COLLECT_SET0);
				break;

			case COLLECT_SET0:
				if (stateTimer.milliseconds() > BlueCloseAutoConstants12Ball.lingerTimeMs) {
					intakeOffTimer.reset();
					intakeOffTimerActive = true;
					setState(AutoState.DRIVE_TO_LAUNCH_4);
				}
				break;

			case DRIVE_TO_LAUNCH_4:
				if (stateChanged) {
					launcherSpinning = true;
					follower.followPath(pathFromSet0ToLaunch, BlueCloseAutoConstants12Ball.defaultSpeed, true);
				}
				if (pathDone()) setState(AutoState.LAUNCH_4);
				break;

			case LAUNCH_4:
				handleLaunch(stateChanged, AutoState.DRIVE_TO_FINAL);
				break;

			// ===== FINAL =====
			case DRIVE_TO_FINAL:
				if (stateChanged) {
					follower.followPath(pathToFinal, BlueCloseAutoConstants12Ball.defaultSpeed, true);
				}
				if (pathDone()) setState(AutoState.FINISHED);
				break;

			case EMERGENCY_DRIVE_TO_FINAL:
				// Path was already set when entering this state from the emergency timer
				if (pathDone()) setState(AutoState.FINISHED);
				break;

			case FINISHED:
				if (stateChanged) {
					intakeRunningState = IntakeState.STOP;
				}
				break;
		}
	}

	protected boolean pathDone() {
		return !follower.isBusy() && stateTimer.milliseconds() > 100;
	}

	protected void handleLaunch(boolean stateChanged, AutoState nextState) {
		if (stateChanged) {
			ballsLaunched = 0;
			currentLaunchSlot = 0;
			initialLaunchDone = false;
			launchTimer.reset();
			recheckTimer.reset();
			follower.holdPoint(BlueCloseAutoConstants12Ball.launchPose);
			sorterController.lockLiftersForLaunch(true);
		}

		// Phase 1: Fire all 3 pits sequentially
		if (!initialLaunchDone) {
			int pitDelay = getPitDelay(currentLaunchSlot);
			if (launchTimer.milliseconds() > pitDelay && currentLaunchSlot < 3) {
				sorterController.forceLaunchSlot(currentLaunchSlot);
				currentLaunchSlot++;
				ballsLaunched++;
				launchTimer.reset();
			}
			if (ballsLaunched >= 3) {
				initialLaunchDone = true;
				launchTimer.reset();
				recheckTimer.reset();
			}
		}

		// Phase 2: During post-launch delay, double-check pits for remaining balls and fire them
		if (initialLaunchDone) {
			// Periodically check for remaining balls using sensor data
			if (recheckTimer.milliseconds() > BlueCloseAutoConstants12Ball.shotDelayMs) {
				sorterController.updateSensors();
				if (sorterController.getCachedBallCount() > 0) {
					// Force-launch ALL pits that still have a ball detected
					for (int i = 0; i < 3; i++) {
						if (sorterController.getCachedColor(i) != CustomSorterController.CustomColor.NULL) {
							sorterController.forceLaunchSlot(i);
						}
					}
					launchTimer.reset(); // Reset timer to allow these extra launches to complete
				}
				recheckTimer.reset();
			}

			// Transition: post-launch delay passed AND no balls remain, or safety timeout
			if ((launchTimer.milliseconds() > BlueCloseAutoConstants12Ball.postLaunchDelayMs
					&& sorterController.getCachedBallCount() == 0)
					|| stateTimer.milliseconds() > BlueCloseAutoConstants12Ball.maxLaunchStateTimeMs) {
				sorterController.lockLiftersForLaunch(false);
				setState(nextState);
			}
		}
	}

	protected int getPitDelay(int pit) {
		switch (pit) {
			case 0: return BlueCloseAutoConstants12Ball.pit0DelayMs;
			case 1: return BlueCloseAutoConstants12Ball.pit1DelayMs;
			case 2: return BlueCloseAutoConstants12Ball.pit2DelayMs;
			default: return BlueCloseAutoConstants12Ball.shotDelayMs;
		}
	}

	protected void setState(AutoState newState) {
		previousState = currentState;
		currentState = newState;
		stateTimer.reset();
	}

	protected void updateTelemetry() {
		telemetryC.addData("State", currentState.toString());
		telemetryC.addData("Runtime", String.format("%.1f s", runtime.seconds()));
		telemetryC.addData("Balls Launched", ballsLaunched + "/3");
		telemetryC.addData("Current Pit", currentLaunchSlot);
		Pose pose = follower.getPose();
		telemetryC.addData("Pose", String.format("(%.1f, %.1f, %.1f°)", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading())));
		telemetryC.update();
	}

	@Override
	public void stop() {
		launcherSpinning = false;
		launcherMotors.setRPM(0);
		intakeRunningState = IntakeState.STOP;
		intakeMotor.setPower(0);
		customThreads.stopLauncherPIDThread();
		customThreads.stopSorterThread();

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
