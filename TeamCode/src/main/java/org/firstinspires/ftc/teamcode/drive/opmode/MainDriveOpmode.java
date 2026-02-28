package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.util.DashboardTelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.drive.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.drive.AutoToTeleDataTransferer;
import org.firstinspires.ftc.teamcode.drive.CustomPIDFController;
import org.firstinspires.ftc.teamcode.drive.CustomThreads;
import org.firstinspires.ftc.teamcode.drive.GamepadEventHandler;
import org.firstinspires.ftc.teamcode.drive.LauncherCalculations;
import org.firstinspires.ftc.teamcode.drive.LocalizationAutoAim;
import org.firstinspires.ftc.teamcode.drive.MDOConstants;
import org.firstinspires.ftc.teamcode.drive.HubInitializer;
import org.firstinspires.ftc.teamcode.drive.CustomAxonServoController;
import org.firstinspires.ftc.teamcode.drive.CustomMotorController;
import org.firstinspires.ftc.teamcode.drive.CustomMotor;
import org.firstinspires.ftc.teamcode.drive.CustomTelemetry;
import org.firstinspires.ftc.teamcode.drive.CustomSorterController;
import org.firstinspires.ftc.teamcode.drive.TurretSubsystem;
import org.firstinspires.ftc.teamcode.drive.ElevationSubsystem;
import org.firstinspires.ftc.teamcode.drive.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.drive.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.DrawbridgeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Drive", group = "!advanced")
public class MainDriveOpmode extends OpMode {
	// Core robot
	AprilTagLocalizer localizer;
	Follower follower;

	// Subsystems
	TurretSubsystem turret;
	ElevationSubsystem elevation;
	LauncherSubsystem launcher;
	IntakeSubsystem intake;
	DrawbridgeSubsystem drawbridge;
	CustomSorterController sorterController;

	// Telemetry
	CustomTelemetry telemetryC;
	static DashboardTelemetryManager telemetryM;

	// Cached per-loop data
	private Double[] launchVectors;

	enum Team {
		RED,
		BLUE
	}

	Team team = Team.BLUE;

	enum StartPosition {
		CLOSE,
		FAR
	}

	StartPosition startPosition = StartPosition.CLOSE;
	boolean hasAutoPose = false;
	ElapsedTime aprilSlowdownTimer = new ElapsedTime();
	ElapsedTime loopTimer = new ElapsedTime();
	double loopTimeMs = 0;
	ElapsedTime sectionTimer = new ElapsedTime();
	double timeCaching = 0, timeUpdate = 0, timeSorter = 0, timeDrive = 0;
	double timeAprilTag = 0, timeServo = 0, timeLauncher = 0, timeIntake = 0, timeTelemetry = 0;

	// Telemetry throttling to reduce overhead
	private int telemetryLoopCounter = 0;
	private static final int TELEMETRY_UPDATE_INTERVAL = 5;

	// Disable camera completely - set to false to skip all camera/AprilTag initialization
	private static final boolean ENABLE_CAMERA = false;

	// Custom functional interface for path building (compatible with API < 24)
	private interface PathChainSupplier {
		PathChain get();
	}

	// In-teleop automated pathing state
	private boolean automatedDriveActive = false;
	private boolean holdingPosition = false;
	private PathChainSupplier launchPositionPath;
	private PathChainSupplier humanPlayerPath;
	private PathChainSupplier gatePositionPath;
	private static final double STICK_OVERRIDE_THRESHOLD = 0.1;

	// Threads
	CustomThreads customThreads;
	private final Object followerLock = new Object();
	boolean pathsBuilt = false;

	// Gamepad event handlers
	GamepadEventHandler gp1Handler;
	GamepadEventHandler gp2Handler;

	@Override
	public void init() {
		telemetryM = DashboardTelemetryManager.create();
		telemetryC = new CustomTelemetry(telemetry, telemetryM);

		// Sorting Initialization
		sorterController = new CustomSorterController(hardwareMap);

		// Get data from autonomous if available
		AutoToTeleDataTransferer dataTransfer = AutoToTeleDataTransferer.getInstance();
		Pose startPose = dataTransfer.getEndPose();
		if (startPose == null) {
			startPose = new Pose(0, 0, 0);
		}
		String teamString = dataTransfer.getAllianceColor();
		if (Objects.equals(teamString, "BLUE")) team = Team.BLUE;
		if (Objects.equals(teamString, "RED")) team = Team.RED;
		if (!Objects.equals(teamString, "UNKNOWN")) {
			buildPaths();
		}

		HubInitializer.initBulkCaching(hardwareMap);

		if (ENABLE_CAMERA) {
			localizer = new AprilTagLocalizer();
			localizer.cameraPosition = new Position(DistanceUnit.INCH,
					MDOConstants.CameraOffset[0], MDOConstants.CameraOffset[1], MDOConstants.CameraOffset[2], 0);
		}

		// ===== SUBSYSTEM INITIALIZATION =====

		// Elevation subsystem
		CustomAxonServoController elevationServo = new CustomAxonServoController(
				hardwareMap,
				new String[]{"elevationServo"},
				new boolean[]{false},
				false,
				new double[]{0, 0, 0},
				null
		);
		elevationServo.setAllowWrapAround(false);
		elevation = new ElevationSubsystem(elevationServo);

		// Turret (azimuth) subsystem
		CustomAxonServoController azimuthServo = new CustomAxonServoController(
				hardwareMap,
				new String[]{"azimuthServo0", "azimuthServo1"},
				new boolean[]{true, true},
				true,
				MDOConstants.AzimuthPIDFConstants,
				"azimuthPosition"
		);
		turret = new TurretSubsystem(azimuthServo);

		if (ENABLE_CAMERA) {
			localizer.initAprilTag(hardwareMap, "Webcam 1");
		}

		// Launcher subsystem
		CustomMotorController launcherMotors = new CustomMotorController(
				hardwareMap,
				new String[]{"launcher0", "launcher1"},
				new boolean[]{true, false},
				new boolean[]{true, false},
				new boolean[]{true, false},
				32,
				new CustomPIDFController(0, 0, 0, 0)
		);
		launcher = new LauncherSubsystem(launcherMotors);

		// Intake subsystem
		CustomMotorController intakeMotor = new CustomMotorController(
				hardwareMap,
				new String[]{"intake"},
				new boolean[]{true},
				false,
				28.0,
				new CustomPIDFController(0, 0, 0, 0)
		);
		intake = new IntakeSubsystem(intakeMotor);

		// Drawbridge subsystem
		CustomMotor drawbridgeMotor = new CustomMotor(hardwareMap, "drawBridge", false, 67, new CustomPIDFController(0, 0, 0, 0));
		drawbridge = new DrawbridgeSubsystem(drawbridgeMotor);

		// ===== FOLLOWER INITIALIZATION =====
		follower = Constants.createFollower(hardwareMap);

		hasAutoPose = (startPose.getX() != 0 || startPose.getY() != 0 || startPose.getHeading() != 0);

		if (hasAutoPose) {
			follower.setStartingPose(startPose);
			telemetryC.addData("Starting Pose from Auto",
					String.format(java.util.Locale.US, "(%.1f, %.1f, %.1f°)",
							startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading())));
		} else {
			follower.setStartingPose(new Pose(0, 0, 0));
			telemetryC.addData("Starting Pose", "Select during init (no auto pose)");
		}
		telemetryC.update();

		follower.startTeleopDrive();

		// ===== THREAD SETUP =====
		aprilSlowdownTimer.reset();
		customThreads = new CustomThreads(follower);
		customThreads.setAzimuthServo(turret.getServoController());
		customThreads.setLauncherMotors(launcher.getMotorController());
		customThreads.setSorterController(sorterController);
		if (ENABLE_CAMERA) {
			customThreads.setAprilTagLocalizer(localizer);
		}

		// ===== GAMEPAD EVENT HANDLER SETUP =====
		gp1Handler = new GamepadEventHandler();
		gp2Handler = new GamepadEventHandler();

		// --- Gamepad 1 (Driver) ---

		// D-pad Down: Go to launch position
		gp1Handler.onPress("goToLaunch", gp -> gp.dpad_down, () -> {
			PathChain pathToFollow = launchPositionPath.get();
			synchronized (followerLock) {
				follower.followPath(pathToFollow, true);
			}
			automatedDriveActive = true;
			holdingPosition = false;
		});

		// D-pad Up: Go to human player station
		gp1Handler.onPress("goToHuman", gp -> gp.dpad_up, () -> {
			PathChain pathToFollow = humanPlayerPath.get();
			synchronized (followerLock) {
				follower.followPath(pathToFollow, true);
			}
			automatedDriveActive = true;
			holdingPosition = false;
		});

		// D-pad Right: Go to gate position
		gp1Handler.onPress("goToGate", gp -> gp.dpad_right, () -> {
			PathChain pathToFollow = gatePositionPath.get();
			synchronized (followerLock) {
				follower.followPath(pathToFollow, true);
			}
			automatedDriveActive = true;
			holdingPosition = false;
		});

		// --- Gamepad 2 (Operator) ---

		// D-pad Down: Toggle launcher spin
		gp2Handler.onDebouncedPress("launcherToggle", gp -> gp.dpad_down, 300, () -> launcher.toggleSpin());

		// D-pad Up: Toggle launcher poop mode
		gp2Handler.onDebouncedPress("launcherPoop", gp -> gp.dpad_up, 300, () -> launcher.togglePoop());

		// Left trigger: Toggle launcher reverse
		gp2Handler.onAnalogPress("launcherReverse", gp -> (double) gp.left_trigger, 0.5, () -> launcher.toggleReverse());

		// D-pad Left/Right: Manual azimuth offset adjustment (150ms debounce)
		gp2Handler.onDebouncedPress("azimuthRight", gp -> gp.dpad_right, 150, () -> turret.adjustAzimuth(1.0));
		gp2Handler.onDebouncedPress("azimuthLeft", gp -> gp.dpad_left, 150, () -> turret.adjustAzimuth(-1.0));

		// Y button: Reset azimuth offset
		gp2Handler.onDebouncedPress("azimuthReset", gp -> gp.y, 150, () -> turret.resetAzimuthOffset());

		// Right bumper: Launch purple ball (650ms cooldown)
		gp2Handler.onDebouncedPress("launchPurple", gp -> gp.right_bumper, 650,
				() -> sorterController.launchCached(CustomSorterController.CustomColor.PURPLE));

		// Left bumper: Launch green ball (650ms cooldown)
		gp2Handler.onDebouncedPress("launchGreen", gp -> gp.left_bumper, 650,
				() -> sorterController.launchCached(CustomSorterController.CustomColor.GREEN));

		// Right trigger: Start rapid fire sequence (edge detection)
		gp2Handler.onAnalogPress("rapidFire", gp -> (double) gp.right_trigger, 0.5,
				() -> launcher.startRapidFire(sorterController));

		// A button: Toggle intake IN
		gp2Handler.onDebouncedPress("intakeIn", gp -> gp.a, 300, () -> intake.toggleIn());

		// B button: Toggle intake OUT
		gp2Handler.onDebouncedPress("intakeOut", gp -> gp.b, 300, () -> intake.toggleOut());

	}

	@SuppressLint("DefaultLocale")
	@Override
	public void init_loop() {
		// Team selection
		if (gamepad1.x) {
			team = Team.BLUE;
		} else if (gamepad1.b) {
			team = Team.RED;
		}

		// Start position selection (only when no auto pose)
		if (!hasAutoPose) {
			if (gamepad1.dpad_up) {
				startPosition = StartPosition.CLOSE;
			} else if (gamepad1.dpad_down) {
				startPosition = StartPosition.FAR;
			}
		}

		telemetryC.addData("Selected Team", team);
		telemetryC.addData("Select Team", "Press X for Blue, B for Red");

		if (hasAutoPose) {
			telemetryC.addData("Start Position", "FROM AUTO (locked)");
		} else {
			telemetryC.addData("Selected Start Position", startPosition);
			telemetryC.addData("Select Position", "D-Pad Up for Close, D-Pad Down for Far");

			// Show the current selected pose values
			double[] selectedPose;
			if (team == Team.RED) {
				selectedPose = (startPosition == StartPosition.CLOSE) ?
						MDOConstants.RedCloseStartPose : MDOConstants.RedFarStartPose;
			} else {
				selectedPose = (startPosition == StartPosition.CLOSE) ?
						MDOConstants.BlueCloseStartPose : MDOConstants.BlueFarStartPose;
			}
			telemetryC.addData("Start Pose Preview",
					String.format("(%.1f, %.1f, %.1f°)", selectedPose[0], selectedPose[1], selectedPose[2]));
		}
		telemetryC.update();
	}

	@Override
	public void start() {
		// Apply selected start position if no auto pose was received
		if (!hasAutoPose) {
			double[] selectedPose;
			if (team == Team.RED) {
				selectedPose = (startPosition == StartPosition.CLOSE) ?
						MDOConstants.RedCloseStartPose : MDOConstants.RedFarStartPose;
			} else {
				selectedPose = (startPosition == StartPosition.CLOSE) ?
						MDOConstants.BlueCloseStartPose : MDOConstants.BlueFarStartPose;
			}
			follower.setStartingPose(new Pose(selectedPose[0], selectedPose[1], Math.toRadians(selectedPose[2])));
		}

		customThreads.startDrawingThread();
		customThreads.startCPUMonThread();
		customThreads.startAzimuthPIDThread();
		// Start the thread that reads color sensors ~20 times/second
		customThreads.startSorterThread();
		// Start the drive thread for more responsive driving
		if (MDOConstants.EnableThreadedDrive) {
			customThreads.startDriveThread();
		}
		// Start the follower update thread - this is KEY for performance!
		// follower.update() is expensive (odometry, path following, motor control)
		if (MDOConstants.EnableThreadedFollowerUpdate) {
			customThreads.startFollowerUpdateThread();
		}
		// Start the AprilTag processing thread to offload vision processing
		if (ENABLE_CAMERA) {
			customThreads.startAprilTagThread();
		}
		// Start the launcher PIDF thread for RPM control
		customThreads.startLauncherPIDThread();
		if (!pathsBuilt) buildPaths();

	}

	@Override
	public void stop() {
		customThreads.stopDrawingThread();
		customThreads.stopCPUMonThread();
		customThreads.stopAzimuthPIDThread();
		// Stop the thread to prevent resource leaks
		customThreads.stopSorterThread();
		// Stop the drive thread
		if (MDOConstants.EnableThreadedDrive) {
			customThreads.stopDriveThread();
		}
		// Stop the follower update thread
		if (MDOConstants.EnableThreadedFollowerUpdate) {
			customThreads.stopFollowerUpdateThread();
		}
		// Stop the AprilTag thread
		if (ENABLE_CAMERA) {
			customThreads.stopAprilTagThread();
			localizer.stopStream();
		}
		// Stop the launcher PID thread
		customThreads.stopLauncherPIDThread();
	}

	@SuppressLint("DefaultLocale")
	@Override
	public void loop() {
		loopTimer.reset();

		// ===== CACHE ALL VALUES AT THE BEGINNING =====
		sectionTimer.reset();

		// Follower/Pose data
		Pose currentPose;
		double poseX, poseY, robotHeadingRad;
		if (MDOConstants.EnableThreadedFollowerUpdate) {
			currentPose = customThreads.getThreadSafePose();
		} else {
			currentPose = follower.getPose();
		}
		if (currentPose == null) {
			currentPose = new Pose(0, 0, 0);
		}
		poseX = currentPose.getX();
		poseY = currentPose.getY();
		robotHeadingRad = currentPose.getHeading();

		// Launch calculations
		boolean isRedSide = (team == Team.RED);
		launchVectors = LauncherCalculations.localizerLauncherCalc(currentPose, (team == Team.RED) ?
				MDOConstants.redTargetLocation : MDOConstants.blueTargetLocation, isRedSide);

		Double[] currentTarget = (team == Team.RED) ? MDOConstants.redTargetLocation : MDOConstants.blueTargetLocation;
		int dynamicRPM = LauncherCalculations.calculateLauncherRPM(currentPose, currentTarget, isRedSide);

		// Gamepad inputs - sticks only (buttons handled by event handlers)
		double gamepad1LeftStickY = gamepad1.left_stick_y;
		double gamepad1LeftStickX = gamepad1.left_stick_x;
		double gamepad1RightStickX = gamepad1.right_stick_x;


		// Cache turret servo data ONCE for telemetry
		double servoPosition = turret.getServoPosition();
		double servoTarget = turret.getServoTarget();

		// CPU data
		double cpuUsage = customThreads.getCpuUsage();
		double cpuTemp = customThreads.getCpuTemp();

		// ===== EMERGENCY STOP CHECK =====
		// Stop OpMode immediately if CPU temperature is critically high
		// Note: CPU heating is from intensive processing (vision, odometry, calculations)
		// NOT from motor current - motors are on separate expansion hub boards
		if (cpuTemp >= MDOConstants.EmergencyStopTemp && cpuTemp > 0) {
			telemetry.addData("EMERGENCY STOP", "");
			telemetry.addData("CPU TEMP TOO HIGH", String.format("%.1f°C", cpuTemp));
			telemetry.addData("Critical Limit", String.format("%.1f°C", MDOConstants.EmergencyStopTemp));
			telemetry.addData("", "OpMode stopping to prevent damage");
			telemetry.update();
			requestOpModeStop();
			return; // Exit loop immediately
		}

		// Cache AprilTag position from the background thread (non-blocking)
		Double[] aprilPose = customThreads.getCachedAprilPose();
		timeCaching = sectionTimer.milliseconds();

		// ===== MAIN LOOP LOGIC =====
		sectionTimer.reset();

		// Only update follower in main thread if threaded update is disabled
		if (!MDOConstants.EnableThreadedFollowerUpdate) {
			follower.update();
		}
		timeUpdate = sectionTimer.milliseconds();

		// ===== SORTER CONTROLLER UPDATES =====
		sectionTimer.reset();
		sorterController.lifterUpdater();
		//sorterController.lightingUpdater();

		// Ball Counting
		// Retrieve the latest ball count from the background thread's cache.
		// This is non-blocking and instant, unlike direct I2C sensor reads.
		int currentBallCount = sorterController.getCachedBallCount();

		timeSorter = sectionTimer.milliseconds();
		// azimuthServo.servoPidLoop() now runs in separate thread

		// ===== DRIVE CONTROL =====
		sectionTimer.reset();

		// Check for manual override - any significant stick input cancels automated pathing/holding
		boolean manualOverride = Math.abs(gamepad1LeftStickY) > STICK_OVERRIDE_THRESHOLD ||
								 Math.abs(gamepad1LeftStickX) > STICK_OVERRIDE_THRESHOLD ||
								 Math.abs(gamepad1RightStickX) > STICK_OVERRIDE_THRESHOLD;

		// ===== AUTOMATED PATHING TRIGGERS (via event handler) =====
		gp1Handler.update(gamepad1);

		// ===== PATH COMPLETION - ENTER HOLD POSITION MODE =====
		// When path completes, it automatically holds position due to holdPoint=true
		// We just track state to know when to allow manual override
		if (automatedDriveActive && !follower.isBusy()) {
			automatedDriveActive = false;
			holdingPosition = true; // Now holding position at destination
		}

		// ===== CANCEL AUTOMATED/HOLD MODE =====
		// Manual stick input or B button releases control back to teleop
		if ((automatedDriveActive || holdingPosition) && (manualOverride || gamepad1.b)) {
			synchronized (followerLock) {
				follower.startTeleopDrive();
			}
			automatedDriveActive = false;
			holdingPosition = false;
		}

		// ===== TELEOP DRIVE (only when not in automated or hold mode) =====
		if (!automatedDriveActive && !holdingPosition) {
			if (MDOConstants.EnableThreadedDrive) {
				// Use threaded drive for maximum responsiveness
				// The drive thread runs at ~200Hz independently of this loop
				customThreads.setDriveInputs(
						-gamepad1LeftStickY,
						-gamepad1LeftStickX,
						gamepad1RightStickX * -0.75,
						MDOConstants.EnableFieldCentricDrive
				);
			} else {
				// Direct drive control (old method)
				follower.setTeleOpDrive(
						-gamepad1LeftStickY,
						-gamepad1LeftStickX,
						gamepad1RightStickX * -0.75,
						MDOConstants.EnableFieldCentricDrive
				);
			}
		}

		timeDrive = sectionTimer.milliseconds();

		// ===== APRILTAG LOCALIZATION =====
		sectionTimer.reset();

		// Update Camera Offset from MDOConstants (allows for live tuning)
		if (localizer != null) {
			localizer.setCameraPose(new Position(DistanceUnit.INCH,
							MDOConstants.CameraOffset[0], MDOConstants.CameraOffset[1], MDOConstants.CameraOffset[2], 0),
					localizer.cameraOrientation);
		}

		// AprilTag pose is cached in background thread at configurable frequency
		// Only update follower pose if we have a valid tag and good confidence
		if (aprilPose != null && MDOConstants.useAprilTags && localizer != null) {
			double decisionMargin = localizer.getDecisionMargin();
			// Use alliance-specific AprilTag heading offset
			double aprilTagHeadingOffset = (team == Team.RED) ? MDOConstants.RedAprilTagHeadingOffset : MDOConstants.BlueAprilTagHeadingOffset;
			if (decisionMargin > 0.8 && aprilSlowdownTimer.milliseconds() > 100) {
				Pose newPose = new Pose(aprilPose[0], aprilPose[1], aprilPose[3] + Math.toRadians(aprilTagHeadingOffset));
				// Use thread-safe method when follower update thread is running
				if (MDOConstants.EnableThreadedFollowerUpdate) {
					customThreads.setThreadSafePose(newPose);
				} else {
					follower.setPose(newPose);
				}
				aprilSlowdownTimer.reset();
			}
		}

		timeAprilTag = sectionTimer.milliseconds();

		// ===== SERVO CONTROL (Turret + Elevation) =====
		sectionTimer.reset();

		turret.update(currentPose, launchVectors, isRedSide);
		elevation.update(currentPose, launchVectors, isRedSide);

		timeServo = sectionTimer.milliseconds();

		// ===== LAUNCHER CONTROL =====
		sectionTimer.reset();

		// All button inputs handled by gp2Handler
		gp2Handler.update(gamepad2);

		// Rapid fire continuation (must run every loop to advance the sequence)
		launcher.updateRapidFire(sorterController);

		// Update launcher motor output
		launcher.update(dynamicRPM);

		timeLauncher = sectionTimer.milliseconds();

		// ===== INTAKE CONTROL =====
		sectionTimer.reset();

		intake.update(currentBallCount);

		timeIntake = sectionTimer.milliseconds();

		// ===== TELEMETRY BLOCK - Throttled to reduce overhead =====
		sectionTimer.reset();

		telemetryLoopCounter++;
		if (telemetryLoopCounter >= TELEMETRY_UPDATE_INTERVAL) {
			telemetryLoopCounter = 0;
			telemetryC.addData("Team", team.toString());
			telemetryC.addData("Active Azimuth Adj", isRedSide ? MDOConstants.RedAzimuthFineAdjustment : MDOConstants.BlueAzimuthFineAdjustment);
			telemetryC.addData("Active Elevation Offset", isRedSide ? MDOConstants.RedElevationOffset : MDOConstants.BlueElevationOffset);
			telemetryC.addData("Active AprilTag Offset", isRedSide ? MDOConstants.RedAprilTagHeadingOffset : MDOConstants.BlueAprilTagHeadingOffset);
			telemetryC.addData("External Heading (deg)", robotHeadingRad);
			telemetryC.addData("Pose X", poseX);
			telemetryC.addData("Pose Y", poseY);

			if (aprilPose != null) {
				telemetryC.addData("X (in) AprilTag", aprilPose[0]);
				telemetryC.addData("Y (in) AprilTag", aprilPose[1]);
				telemetryC.addData("AprilTag ID", (int) aprilPose[4].doubleValue());
				telemetryC.addData("AprilTag Heading (rad)", aprilPose[3]);
				telemetryC.addData("Decision Margin", localizer.getDecisionMargin());
			} else {
				telemetryC.addData("X (in) AprilTag", "No Tag");
				telemetryC.addData("Y (in) AprilTag", "No Tag");
			}

			if (launchVectors != null) {
				telemetryC.addData("Launch Elevation (deg)", elevation.getLaunchElevationDeg());
				telemetryC.addData("Elevation Servo Pos", elevation.getElevationServoFinal());
				telemetryC.addData("--- AZIMUTH DEBUG ---", "");
				telemetryC.addData("Robot Heading (raw)", String.format("%.1f", turret.getRobotFieldRelativeAzimuthDeg()));
				telemetryC.addData("Target Azimuth (raw)", String.format("%.1f", turret.getLaunchAzimuthDeg()));
				telemetryC.addData("Final Azimuth (deg)", String.format("%.2f", turret.getFinalAzimuthDeg()));
				telemetryC.addData("Manual Azimuth Offset", turret.getManualAzimuthOffset());
				telemetryC.addData("Servo Pos (actual)", String.format("%.3f", servoPosition));
				telemetryC.addData("Servo Target (actual)", String.format("%.3f", servoTarget));
			} else {
				telemetryC.addData("Launch Vectors", "Target Unreachable");
			}

			telemetryC.addData("Launcher RPM", launcher.getRPM());
			telemetryC.addData("Launcher Debug", launcher.getDebugString());
			telemetryC.addData("Launcher Pressure", launcher.getPIDOutput());
			telemetryC.addData("launcherRunning", launcher.isSpinning());
			telemetryC.addData("Launcher Reverse", launcher.isReverse() ? "ON (" + MDOConstants.LauncherReverseRPM + " RPM)" : "OFF");

			// Distance to target and dynamic RPM
			if (currentTarget != null) {
				Double[] robotPos = {poseX, poseY, 10.0};
				double distanceToTarget = LocalizationAutoAim.getDistance(robotPos, currentTarget);
				telemetryC.addData("Distance to Target (in)", String.format("%.1f", distanceToTarget));
				telemetryC.addData("Dynamic RPM (Target)", dynamicRPM);
			}

			telemetryC.addData("Intake State", intake.getState());
			telemetryC.addData("Ball Count", currentBallCount);

			// Cached color sensor data (updated in background thread)
			telemetryC.addData("Color 0", sorterController.getCachedColor(0));
			telemetryC.addData("Color 1", sorterController.getCachedColor(1));
			telemetryC.addData("Color 2", sorterController.getCachedColor(2));

			// ===== COLOR SENSOR DEBUG TELEMETRY =====
			// Shows raw RGB and HSV values from all 6 color sensors (updated in background thread)
			// Sensors 0,1 = Pit 0 | Sensors 4,5 = Pit 1 | Sensors 2,3 = Pit 2
			telemetryC.addData("--- Color Sensor Debug ---", "");
			for (int s = 0; s < 6; s++) {
				telemetryC.addData("Sensor " + s, sorterController.getDebugString(s));
			}
			// Show classification thresholds for reference
			telemetryC.addData("Thresholds", String.format("MinBright:%.2f MinSat:%.2f GreenH:[%.0f-%.0f] PurpleH:[%.0f-%.0f]",
					MDOConstants.ColorMinBrightness, MDOConstants.ColorMinSaturation,
					MDOConstants.GreenHueMin, MDOConstants.GreenHueMax,
					MDOConstants.PurpleHueMin, MDOConstants.PurpleHueMax));


			// ===== CPU USAGE/TEMP TELEMETRY =====
			telemetryC.addData("CPU Usage (%)", cpuUsage);
			telemetryC.addData("CPU Temp (°C)", cpuTemp);

			// ===== THERMAL WARNING =====
			// Warn if approaching emergency stop temperature (within 5°C)
			if (cpuTemp >= MDOConstants.EmergencyStopTemp - 5 && cpuTemp > 0) {
				telemetryC.addData("🚨 CRITICAL TEMP 🚨", String.format("%.1f°C (Stop at %.1f°C)",
						cpuTemp, MDOConstants.EmergencyStopTemp));
			}


			// ===== PERFORMANCE TIMING TELEMETRY =====
			telemetryC.addData("Loop Time (ms)", loopTimeMs);

			double totalTrackedTime = timeCaching + timeUpdate + timeSorter + timeDrive +
					timeAprilTag + timeServo + timeLauncher + timeTelemetry;
			double unaccountedTime = loopTimeMs - totalTrackedTime;

			telemetryC.addData("--- Performance Breakdown ---", "");
			telemetryC.addData("Caching", String.format("%.2f ms", timeCaching));
			telemetryC.addData("Update/PIDF", String.format("%.2f ms", timeUpdate));
			telemetryC.addData("Sorter Updates", String.format("%.2f ms", timeSorter));
			telemetryC.addData("Drive Control", String.format("%.2f ms", timeDrive));
			telemetryC.addData("AprilTag", String.format("%.2f ms", timeAprilTag));
			telemetryC.addData("Servo Control", String.format("%.2f ms", timeServo));
			telemetryC.addData("Launcher Control", String.format("%.2f ms", timeLauncher));
			telemetryC.addData("Telemetry", String.format("%.2f ms", timeTelemetry));
			telemetryC.addData("--- Summary ---", "");
			telemetryC.addData("Total Tracked", String.format("%.2f ms", totalTrackedTime));
			telemetryC.addData("Unaccounted Time", String.format("%.2f ms (%.1f%%)",
					unaccountedTime, (unaccountedTime / loopTimeMs) * 100));
			telemetryC.addData("lifterTimer", lifterAutoLaunchTimer.milliseconds());

			// Update Telemetry
			telemetryC.update();
		}

		timeTelemetry = sectionTimer.milliseconds();
		loopTimeMs = loopTimer.milliseconds();
	}

	public void buildPaths() {
		// Initialize lazy path suppliers for in-teleop automated pathing
		// These use Supplier<PathChain> so the path is built from current position when triggered
		launchPositionPath = () -> {
			Pose targetPose = (team == Team.RED) ? MDOConstants.redCloseLaunchLocation : MDOConstants.blueCloseLaunchLocation;
			return follower.pathBuilder()
					.addPath(new Path(new BezierLine(
							follower::getPose,
							targetPose
					)))
					.setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getHeading())
					.build();
		};

		humanPlayerPath = () -> {
			Pose targetPose = (team == Team.RED) ? MDOConstants.redHumanLocation : MDOConstants.blueHumanLocation;
			return follower.pathBuilder()
					.addPath(new Path(new BezierLine(
							follower::getPose,
							targetPose
					)))
					.setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getHeading())
					.build();
		};

		gatePositionPath = () -> {
			Pose targetPose = (team == Team.RED) ? MDOConstants.redGateLocation : MDOConstants.blueGateLocation;
			return follower.pathBuilder()
					.addPath(new Path(new BezierLine(
							follower::getPose,
							targetPose
					)))
					.setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getHeading())
					.build();
		};
		pathsBuilt = true;
	}
}
