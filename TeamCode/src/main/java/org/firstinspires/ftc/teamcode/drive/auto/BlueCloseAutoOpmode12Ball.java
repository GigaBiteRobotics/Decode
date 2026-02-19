package org.firstinspires.ftc.teamcode.drive.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
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
import org.firstinspires.ftc.teamcode.drive.RobotCoreCustom;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Close Auto 12 Ball", group = "Auto")
public class BlueCloseAutoOpmode12Ball extends OpMode {

	// ===== STATE MACHINE =====
	// Sequence (4 launches, 12 balls):
	// 1. Launch preloaded
	// 2. Pickup Set 1 -> Launch
	// 3. Pickup from gate -> Launch
	// 4. Pickup from gate -> Launch
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
		DRIVE_TO_GATE_PICKUP_1,
		COLLECT_GATE_1,
		DRIVE_TO_LAUNCH_3,
		LAUNCH_3,
		// Launch 4: Pickup from gate -> Launch
		DRIVE_TO_GATE_PICKUP_2,
		COLLECT_GATE_2,
		DRIVE_TO_LAUNCH_4,
		LAUNCH_4,
		// Final
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
	protected RobotCoreCustom.CustomTelemetry telemetryC;
	protected static TelemetryManager telemetryM;

	// ===== PATHS =====
	protected PathChain pathToLaunch;
	protected PathChain pathToSet1Lineup, pathToSet1Pickup, pathFromSet1ToLaunch;
	protected PathChain pathToGatePickup, pathFromGateToLaunch;
	protected PathChain pathToFinal;

	// ===== LAUNCH TRACKING =====
	protected int ballsLaunched = 0;
	protected int currentLaunchSlot = 0;

	@Override
	public void init() {
		// Initialize telemetry
		telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
		telemetryC = new RobotCoreCustom.CustomTelemetry(telemetry, telemetryM);
		telemetryC.addData("Status", "Initializing...");
		telemetryC.update();

		// Initialize follower with Pedro Pathing
		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(BlueCloseAutoConstants12Ball.startPose);

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
			BlueCloseAutoConstants12Ball.startPose.getX(),
			BlueCloseAutoConstants12Ball.startPose.getY(),
			Math.toDegrees(BlueCloseAutoConstants12Ball.startPose.getHeading())));
		telemetryC.update();
	}

	protected void buildPaths() {
		// Start to launch
		pathToLaunch = buildPath(BlueCloseAutoConstants12Ball.startPose, BlueCloseAutoConstants12Ball.launchPose);

		// Set 1 paths
		pathToSet1Lineup = buildPath(BlueCloseAutoConstants12Ball.launchPose, BlueCloseAutoConstants12Ball.set1LineupPose);
		pathToSet1Pickup = buildPath(BlueCloseAutoConstants12Ball.set1LineupPose, BlueCloseAutoConstants12Ball.set1PickupPose);
		pathFromSet1ToLaunch = buildPath(BlueCloseAutoConstants12Ball.set1PickupPose, BlueCloseAutoConstants12Ball.launchPose);

		// Gate paths
		pathToGatePickup = buildPath(BlueCloseAutoConstants12Ball.launchPose, BlueCloseAutoConstants12Ball.gatePickupPose);
		pathFromGateToLaunch = buildPath(BlueCloseAutoConstants12Ball.gatePickupPose, BlueCloseAutoConstants12Ball.launchPose);

		// Final
		pathToFinal = buildPath(BlueCloseAutoConstants12Ball.launchPose, BlueCloseAutoConstants12Ball.finalPose);
	}

	protected PathChain buildPath(Pose from, Pose to) {
		return follower.pathBuilder()
			.addPath(new BezierLine(from, to))
			.setLinearHeadingInterpolation(from.getHeading(), to.getHeading())
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
				handleLaunch(stateChanged, AutoState.DRIVE_TO_GATE_PICKUP_1);
				break;

			// ===== LAUNCH 3: Gate Pickup -> Launch =====
			case DRIVE_TO_GATE_PICKUP_1:
				if (stateChanged) {
					intakeRunningState = IntakeState.IN;
					follower.followPath(pathToGatePickup, BlueCloseAutoConstants12Ball.pickupSpeed, true);
				}
				if (pathDone()) setState(AutoState.COLLECT_GATE_1);
				break;

			case COLLECT_GATE_1:
				if (stateTimer.milliseconds() > BlueCloseAutoConstants12Ball.lingerTimeMs) {
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
				handleLaunch(stateChanged, AutoState.DRIVE_TO_GATE_PICKUP_2);
				break;

			// ===== LAUNCH 4: Gate Pickup -> Launch =====
			case DRIVE_TO_GATE_PICKUP_2:
				if (stateChanged) {
					intakeRunningState = IntakeState.IN;
					follower.followPath(pathToGatePickup, BlueCloseAutoConstants12Ball.pickupSpeed, true);
				}
				if (pathDone()) setState(AutoState.COLLECT_GATE_2);
				break;

			case COLLECT_GATE_2:
				if (stateTimer.milliseconds() > BlueCloseAutoConstants12Ball.lingerTimeMs) {
					intakeOffTimer.reset();
					intakeOffTimerActive = true;
					setState(AutoState.DRIVE_TO_LAUNCH_4);
				}
				break;

			case DRIVE_TO_LAUNCH_4:
				if (stateChanged) {
					launcherSpinning = true;
					follower.followPath(pathFromGateToLaunch, BlueCloseAutoConstants12Ball.defaultSpeed, true);
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
			launchTimer.reset();
			follower.holdPoint(BlueCloseAutoConstants12Ball.launchPose);
			sorterController.lockLiftersForLaunch(true);
		}

		int pitDelay = getPitDelay(currentLaunchSlot);
		if (launchTimer.milliseconds() > pitDelay && currentLaunchSlot < 3) {
			sorterController.forceLaunchSlot(currentLaunchSlot);
			currentLaunchSlot++;
			ballsLaunched++;
			launchTimer.reset();
		}

		if (ballsLaunched >= 3 && launchTimer.milliseconds() > BlueCloseAutoConstants12Ball.postLaunchDelayMs) {
			sorterController.lockLiftersForLaunch(false);
			setState(nextState);
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
