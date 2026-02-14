package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.drive.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.drive.AutoToTeleDataTransferer;
import org.firstinspires.ftc.teamcode.drive.CustomPIDFController;
import org.firstinspires.ftc.teamcode.drive.CustomThreads;
import org.firstinspires.ftc.teamcode.drive.LocalizationAutoAim;
import org.firstinspires.ftc.teamcode.drive.MDOConstants;
import org.firstinspires.ftc.teamcode.drive.RobotCoreCustom;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Drive", group = "!advanced")
public class MainDriveOpmode extends OpMode {
	AprilTagLocalizer localizer;
	//IMU imuEX;
	RobotCoreCustom robotCoreCustom;
	Follower follower;
	RobotCoreCustom.CustomAxonServoController elevationServo;
	RobotCoreCustom.CustomAxonServoController azimuthServo;
	Double launchElevationDeg = 0.0;
	double elevationServoTarget = 0.0;
	RobotCoreCustom.CustomMotorController launcherMotors, intakeMotor;
	RobotCoreCustom.CustomMotor drawbridgeMotor;
	Double[] lastAprilLocalization = null;
	double targetPower = 0.0;
	double elevationServoFinal = 0;
	double manualAzimuthOffset = 0.0; // Manual azimuth adjustment via D-pad
	ElapsedTime gamepadTimer = new ElapsedTime();
	ElapsedTime azimuthAdjustTimer = new ElapsedTime(); // Timer for D-pad azimuth adjustment
	RobotCoreCustom.CustomTelemetry telemetryC;
	RobotCoreCustom.CustomSorterController sorterController;
	boolean launcherSpinning = false;
	private Double[] launchVectors;
	int prevBallCount = 0;

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
	ElapsedTime lifterAutoLaunchTimer = new ElapsedTime();
	ElapsedTime aprilSlowdownTimer = new ElapsedTime();
	ElapsedTime loopTimer = new ElapsedTime();
	ElapsedTime intakeInputTimer = new ElapsedTime();
	double loopTimeMs = 0;
	ElapsedTime sectionTimer = new ElapsedTime();
	double timeCaching = 0, timeUpdate = 0, timeSorter = 0, timeDrive = 0;
	double timeAprilTag = 0, timeServo = 0, timeLauncher = 0, timeIntake = 0, timeTelemetry = 0;

	Double launchAzimuthDeg = 0.0;
	Double fieldRelativeAzimuthDeg = 0.0;
	Double robotFieldRelativeAzimuthDeg = 0.0;
	Double finalAzimuthDeg = 0.0;


	static TelemetryManager telemetryM;

	enum intakeState {
		IN,
		OUT,
		STOP
	}

	intakeState intakeRunningState = intakeState.STOP;

	// Telemetry throttling to reduce overhead
	private int telemetryLoopCounter = 0;
	private static final int TELEMETRY_UPDATE_INTERVAL = 5; // Update telemetry every N loops (higher = less lag)

	// Rapid fire state machine
	private boolean rapidFireActive = false;
	private int rapidFireIndex = 0; // Current index in RapidFireOrder
	private final ElapsedTime rapidFireTimer = new ElapsedTime();
	private boolean rapidFireTriggerWasPressed = false; // For edge detection

	// Disable camera completely - set to false to skip all camera/AprilTag initialization
	private static final boolean ENABLE_CAMERA = false;

	// Threads
	// Handles background tasks like sensor reading and servo PID to keep the main loop fast
	CustomThreads customThreads;

	@Override
	public void init() {
		telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
		telemetryC = new RobotCoreCustom.CustomTelemetry(telemetry, telemetryM);

		// Sorting Initialization
		// Initializes servos and color sensors for the ball sorting mechanism
		sorterController = new RobotCoreCustom.CustomSorterController(hardwareMap);

		// Get data from autonomous if available
		AutoToTeleDataTransferer dataTransfer = AutoToTeleDataTransferer.getInstance();
		Pose startPose = dataTransfer.getEndPose();
		if (startPose == null) {
			startPose = new Pose(0, 0, 0);
		}

		robotCoreCustom = new RobotCoreCustom(hardwareMap, follower);

		if (ENABLE_CAMERA) {
			localizer = new AprilTagLocalizer();

			// Apply initial value
			localizer.cameraPosition = new Position(DistanceUnit.INCH,
					MDOConstants.CameraOffset[0], MDOConstants.CameraOffset[1], MDOConstants.CameraOffset[2], 0);
		}

		elevationServo = new RobotCoreCustom.CustomAxonServoController(
				hardwareMap,
				new String[]{"elevationServo"},
				new boolean[]{false},
				false,
				new double[]{0, 0, 0},
				null
		);
		azimuthServo = new RobotCoreCustom.CustomAxonServoController(
				hardwareMap,
				new String[]{"azimuthServo0", "azimuthServo1"},
				new boolean[]{true, true},
				true,
				MDOConstants.AzimuthPIDFConstants,
				"azimuthPosition"
		);
		if (ENABLE_CAMERA) {
			localizer.initAprilTag(hardwareMap, "Webcam 1");
		}

		launcherMotors = new RobotCoreCustom.CustomMotorController(
				hardwareMap,
				new String[]{"launcher0", "launcher1"},
				new boolean[]{true, false}, // launcher1 is reversed
				new boolean[]{true, false}, // encoder reverse map: launcher1 encoder is reversed
				true, // has encoders
				28.0, // ticks per rev
				new CustomPIDFController(0, 0, 0, 0)
		);
		intakeMotor = new RobotCoreCustom.CustomMotorController(
				hardwareMap,
				new String[]{"intake"},
				new boolean[]{true},
				false,
				28.0,
				new CustomPIDFController(0, 0, 0, 0)
		);
		drawbridgeMotor = new RobotCoreCustom.CustomMotor(hardwareMap, "drawBridge", false, 67, new CustomPIDFController(0, 0, 0, 0));

		follower = Constants.createFollower(hardwareMap);

		// Check if we have a pose from auto
		hasAutoPose = (startPose.getX() != 0 || startPose.getY() != 0 || startPose.getHeading() != 0);

		// Use pose from auto if available, otherwise start position will be set in start() based on selection
		if (hasAutoPose) {
			follower.setStartingPose(startPose);
			telemetryC.addData("Starting Pose from Auto",
					String.format("(%.1f, %.1f, %.1f°)",
							startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading())));
		} else {
			// Temporary default, will be updated in start() based on selection
			follower.setStartingPose(new Pose(0, 0, 0));
			telemetryC.addData("Starting Pose", "Select during init (no auto pose)");
		}
		telemetryC.update();

		follower.startTeleopDrive();
		gamepadTimer.reset();
		aprilSlowdownTimer.reset();
		lifterAutoLaunchTimer.reset();
		intakeInputTimer.reset();
		customThreads = new CustomThreads(robotCoreCustom, follower);
		customThreads.setAzimuthServo(azimuthServo);
		customThreads.setLauncherMotors(launcherMotors);
		// Pass the sorter controller to the thread manager so it can update sensors in the background
		customThreads.setSorterController(sorterController);
		// Pass the AprilTag localizer to process detections in background
		if (ENABLE_CAMERA) {
			customThreads.setAprilTagLocalizer(localizer);
		}

	}

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
		// Use thread-safe pose when follower update thread is running to prevent race conditions
		if (MDOConstants.EnableThreadedFollowerUpdate) {
			currentPose = customThreads.getThreadSafePose();
		} else {
			currentPose = follower.getPose();
		}
		if (currentPose == null) {
			// Pose not yet available, use default values and skip this loop iteration
			currentPose = new Pose(0, 0, 0);
		}
		poseX = currentPose.getX();
		poseY = currentPose.getY();
		robotHeadingRad = currentPose.getHeading();
		robotFieldRelativeAzimuthDeg = Math.toDegrees(robotHeadingRad);


		// Launch calculations - pass alliance side for side-specific tuning
		// Use the already-fetched thread-safe pose instead of accessing follower.getPose() again
		boolean isRedSide = (team == Team.RED);
		launchVectors = RobotCoreCustom.localizerLauncherCalc(currentPose, (team == Team.RED) ?
				MDOConstants.redTargetLocation : MDOConstants.blueTargetLocation, isRedSide);

		// Current target and dynamic RPM calculation
		Double[] currentTarget = (team == Team.RED) ? MDOConstants.redTargetLocation : MDOConstants.blueTargetLocation;
		int dynamicRPM = RobotCoreCustom.calculateLauncherRPM(currentPose, currentTarget, isRedSide);

		// Launcher data
		double launcherRPM = launcherMotors.getAverageRPM();

		// Gamepad inputs - all sticks
		double gamepad1LeftStickY = gamepad1.left_stick_y;
		double gamepad1LeftStickX = gamepad1.left_stick_x;
		double gamepad1RightStickX = gamepad1.right_stick_x;

		// Timer values
		double gamepadTimerMs = gamepadTimer.milliseconds();

		// Cache azimuth servo data ONCE to avoid hardware I/O in telemetry
		double servoPosition = azimuthServo.getPosition();
		double servoTarget = azimuthServo.getTargetPosition();

		// Pre-format CPU strings to avoid repeated String.format() in telemetry (expensive operation)
		double cpuUsage = customThreads.getCpuUsage();
		double cpuTemp = customThreads.getCpuTemp();

		// ===== EMERGENCY STOP CHECK =====
		// Stop OpMode immediately if CPU temperature is critically high
		// Note: CPU heating is from intensive processing (vision, odometry, calculations)
		// NOT from motor current - motors are on separate expansion hub boards
		if (cpuTemp >= MDOConstants.EmergencyStopTemp && cpuTemp > 0) {
			telemetry.addData("🚨 EMERGENCY STOP 🚨", "");
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

		// ===== SERVO CONTROL =====
		sectionTimer.reset();

		aimingLoop();

		timeServo = sectionTimer.milliseconds();

		// ===== LAUNCHER CONTROL =====
		sectionTimer.reset();

		if (gamepad2.dpad_down && gamepadTimerMs > 300) {
			gamepadTimer.reset();
			if (!launcherSpinning) {
				targetPower = MDOConstants.launchPower;
				launcherSpinning = true;
			} else {
				targetPower = 0;
				launcherSpinning = false;
			}
		}

		// Manual azimuth offset adjustment using D-pad left/right
		// Adjusts in small increments (1 degree) with a 150ms delay between adjustments
		if (azimuthAdjustTimer.milliseconds() > 150) {
			if (gamepad2.dpad_right) {
				manualAzimuthOffset += 1.0; // Increase azimuth offset by 1 degree
				azimuthAdjustTimer.reset();
			} else if (gamepad2.dpad_left) {
				manualAzimuthOffset -= 1.0; // Decrease azimuth offset by 1 degree
				azimuthAdjustTimer.reset();
			}
			// Reset manual offset with Y button
			if (gamepad2.y) {
				manualAzimuthOffset = 0.0;
				azimuthAdjustTimer.reset();
			}
		}

		if (gamepad2.right_bumper && lifterAutoLaunchTimer.milliseconds() > 650) { // launch purple
			lifterAutoLaunchTimer.reset();
			// Use launchCached to avoid blocking the loop with sensor reads during launch
			sorterController.launchCached(RobotCoreCustom.CustomSorterController.CustomColor.PURPLE);
		}
		if (gamepad2.left_bumper && lifterAutoLaunchTimer.milliseconds() > 650) {
			lifterAutoLaunchTimer.reset();
			// Use launchCached to avoid blocking the loop with sensor reads during launch
			sorterController.launchCached(RobotCoreCustom.CustomSorterController.CustomColor.GREEN);
		}

		// ===== RAPID FIRE LOGIC =====
		// Fires balls from pits in configurable order with settable delay between shots
		// Triggered by holding gamepad2.right_trigger
		boolean rapidFireTriggerPressed = gamepad2.right_trigger > 0.5;

		// Edge detection: Start rapid fire sequence when trigger is first pressed
		if (rapidFireTriggerPressed && !rapidFireTriggerWasPressed && !rapidFireActive) {
			rapidFireActive = true;
			rapidFireIndex = 0;
			rapidFireTimer.reset();
			// Fire first ball immediately
			int pitToFire = MDOConstants.RapidFireOrder[rapidFireIndex];
			sorterController.launchFromPit(pitToFire);
			rapidFireIndex++;
		}
		rapidFireTriggerWasPressed = rapidFireTriggerPressed;

		// Continue rapid fire sequence if active
		if (rapidFireActive && rapidFireIndex < MDOConstants.RapidFireOrder.length) {
			if (rapidFireTimer.milliseconds() >= MDOConstants.RapidFireDelayMs) {
				int pitToFire = MDOConstants.RapidFireOrder[rapidFireIndex];
				sorterController.launchFromPit(pitToFire);
				rapidFireIndex++;
				rapidFireTimer.reset();
			}
		}

		// End rapid fire when all pits have been fired
		if (rapidFireIndex >= MDOConstants.RapidFireOrder.length) {
			rapidFireActive = false;
		}

		//launcherMotors.setPower(launcherSpinning ? MDOConstants.launchPower : 0);

		// Clamp target power
		targetPower = Math.max(-1.0, Math.min(1.0, targetPower));
		launcherMotors.setRPM(launcherSpinning ? dynamicRPM : 0);
		launcherMotors.setPIDFController(MDOConstants.LauncherPIDF);

		timeLauncher = sectionTimer.milliseconds();

		// ===== INTAKE CONTROL =====
		sectionTimer.reset();

		// Auto-Stop Logic:
		// Automatically turns off the intake when the 3rd ball is collected to save power.
		// Logic: If intake is ON AND we have 3+ balls AND we didn't have 3 balls last loop.
		// This allows the driver to manually turn the intake back ON even if 3 balls are held.
		if (intakeRunningState == intakeState.IN && currentBallCount >= 3 && prevBallCount < 3) {
			intakeRunningState = intakeState.STOP;
		}
		prevBallCount = currentBallCount;

		if (intakeInputTimer.milliseconds() > 300) {
			if (gamepad2.a) {
				intakeInputTimer.reset();
				if (intakeRunningState == intakeState.IN) {
					intakeRunningState = intakeState.STOP;
				} else {
					intakeRunningState = intakeState.IN;
				}
			} else if (gamepad2.b) {
				intakeInputTimer.reset();
				if (intakeRunningState == intakeState.OUT) {
					intakeRunningState = intakeState.STOP;
				} else {
					intakeRunningState = intakeState.OUT;
				}
			}
		}

		switch (intakeRunningState) {
			case IN:
				intakeMotor.setPower(1);
				break;
			case OUT:
				intakeMotor.setPower(-1);
				break;
			case STOP:
			default:
				intakeMotor.setPower(0);
				break;
		}
		timeIntake = sectionTimer.milliseconds();

		// ===== TELEMETRY BLOCK - Throttled to reduce overhead =====
		sectionTimer.reset();

		telemetryLoopCounter++;
		if (telemetryLoopCounter >= TELEMETRY_UPDATE_INTERVAL) {
			telemetryLoopCounter = 0; // Reset counter
			telemetryC.addData("Team", team.toString());
			// Show which alliance-specific values are being used (isRedSide already defined above)
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
				telemetryC.addData("Launch Elevation (deg)", launchElevationDeg);
				telemetryC.addData("Elevation Servo Pos", elevationServoFinal);
				telemetryC.addData("--- AZIMUTH DEBUG ---", "");
				telemetryC.addData("Robot Heading (raw)", String.format("%.1f", robotFieldRelativeAzimuthDeg));
				telemetryC.addData("Target Azimuth (raw)", String.format("%.1f", launchAzimuthDeg));
				telemetryC.addData("Final Azimuth (deg)", String.format("%.2f", finalAzimuthDeg));
				telemetryC.addData("Manual Azimuth Offset", manualAzimuthOffset);
				telemetryC.addData("Servo Pos (actual)", String.format("%.3f", servoPosition));
				telemetryC.addData("Servo Target (actual)", String.format("%.3f", servoTarget));
			} else {
				telemetryC.addData("Launch Vectors", "Target Unreachable");
			}

			telemetryC.addData("Launcher RPM", launcherRPM);
			telemetryC.addData("Launcher Pressure", launcherMotors.getPIDOutput());
			telemetryC.addData("launcherRunning", launcherSpinning);

			// Distance to target and dynamic RPM
			if (currentTarget != null) {
				// Use already-cached pose values instead of calling follower.getPose() again
				Double[] robotPos = {poseX, poseY, 10.0};
				double distanceToTarget = LocalizationAutoAim.getDistance(robotPos, currentTarget);
				telemetryC.addData("Distance to Target (in)", String.format("%.1f", distanceToTarget));
				telemetryC.addData("Dynamic RPM (Target)", dynamicRPM);
			}

			telemetryC.addData("Intake State", intakeRunningState);
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

	public void aimingLoop() {
		// Calculate elevation angle for launch
		// start off with IMU offset
		// Pre-calculate launch vector conversions if available
		boolean hasValidLaunchVectors = (launchVectors != null);

		// Get alliance-specific offsets
		boolean isRedSide = (team == Team.RED);
		double elevationOffset = isRedSide ? MDOConstants.RedElevationOffset : MDOConstants.BlueElevationOffset;
		double azimuthFineAdjust = isRedSide ? MDOConstants.RedAzimuthFineAdjustment : MDOConstants.BlueAzimuthFineAdjustment;

		if (hasValidLaunchVectors) {
			launchAzimuthDeg = Math.toDegrees(launchVectors[1]);
			// Elevation is already calculated as servo position in LocalizationAutoAim, not radians
			launchElevationDeg = launchVectors[0];
		}

		if (launchElevationDeg != null) {
			// Use alliance-specific elevation offset combined with shared offset
			elevationServoTarget = (launchElevationDeg + MDOConstants.ElevationOffset + elevationOffset) * MDOConstants.ElevationMultiplier;
			// Fix clamp logic to correctly bound between -0.4 and 1.0
			elevationServoFinal = Math.max(-0.4, Math.min(1.0, elevationServoTarget));
			elevationServo.setPosition(elevationServoFinal);
		}

		// ===== AZIMUTH CONTROL =====
		// Calculates the turret angle needed to aim at the target

		if (MDOConstants.EnableTurretIMUCorrection && MDOConstants.EnableTurret) {
			if (MDOConstants.EnableLauncherCalcAzimuth && hasValidLaunchVectors) {
				// Calculate robot-relative angle to target
				double robotHeadingDeg = robotFieldRelativeAzimuthDeg;
				double targetAzimuthDeg = launchAzimuthDeg;

				// Calculate turret angle with all offsets
				finalAzimuthDeg = robotHeadingDeg - targetAzimuthDeg
						+ MDOConstants.AzimuthIMUOffset
						+ MDOConstants.AzimuthFineAdjustment
						+ azimuthFineAdjust
						+ manualAzimuthOffset;
			} else {
				// When not using launcher calc, maintain heading-based orientation
				double robotHeadingDeg = robotFieldRelativeAzimuthDeg;
				finalAzimuthDeg = (robotHeadingDeg + MDOConstants.AzimuthIMUOffset + manualAzimuthOffset)
						* MDOConstants.AzimuthMultiplier;
			}
		}
		// Normalize finalAzimuthDeg to [-180, 180] range to find shortest path
		// This allows the turret to wrap around instead of hitting limits
		while (finalAzimuthDeg > 180.0) {
			finalAzimuthDeg -= 360.0;
		}
		while (finalAzimuthDeg < -180.0) {
			finalAzimuthDeg += 360.0;
		}

		// Convert degrees to servo position [-1, 1] range
		// The servo controller expects: -1 = 0°, 0 = 180°, +1 = 360°
		// So we divide by 180 to map degrees to the normalized range
		// After normalization, finalAzimuthDeg is in [-180, 180] which maps to [-1, 1]
		double servoPosition = -finalAzimuthDeg / 180.0;



		// Update PID coefficients if changed via Panels (has built-in change detection)
		azimuthServo.setPIDCoefficients(MDOConstants.AzimuthPIDFConstants);

		// Set Azimuth Servos with mapped value (-1 to 1)
		if (MDOConstants.EnableTurret) {
			azimuthServo.setPosition(servoPosition);
		} else {
			azimuthServo.setPosition(0.0);
		}

		// Gear ratios: 96:20 then 25:120 = combined 1:1
    }
}
