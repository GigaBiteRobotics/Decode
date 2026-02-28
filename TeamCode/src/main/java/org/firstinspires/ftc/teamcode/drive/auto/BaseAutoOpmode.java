package org.firstinspires.ftc.teamcode.drive.auto;

import org.firstinspires.ftc.teamcode.util.DashboardTelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.drive.AutoToTeleDataTransferer;
import org.firstinspires.ftc.teamcode.drive.CustomPIDFController;
import org.firstinspires.ftc.teamcode.drive.HubInitializer;
import org.firstinspires.ftc.teamcode.drive.CustomAxonServoController;
import org.firstinspires.ftc.teamcode.drive.CustomMotorController;
import org.firstinspires.ftc.teamcode.drive.CustomMotor;
import org.firstinspires.ftc.teamcode.drive.CustomTelemetry;
import org.firstinspires.ftc.teamcode.drive.CustomSorterController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BaseAuto(DO NOT USE)", group = "z")
public class BaseAutoOpmode extends OpMode {

	// Autonomous state machine
	protected enum AutoState {
		IDLE,
		PATH_1,
		PATH_2,
		PATH_3,
		PATH_4,
		PATH_5,
		ACTION_1,
		ACTION_2,
		ACTION_3,
		ACTION_4,
		ACTION_5,
		WAIT,
		FINISHED
	}
	protected AutoState currentState = AutoState.IDLE;
	protected AutoState previousState = AutoState.IDLE;

	// Robot core components
	protected Follower follower;
	protected AprilTagLocalizer aprilTagLocalizer;

	// Servos
	protected Servo elevationServo, azimuthServo0, azimuthServo1;

	// Motors
	protected CustomMotor launcher;

	// Sensors
	protected ColorSensor colorSensor;

	// Timers
	protected ElapsedTime runtime;
	protected ElapsedTime autoTimer;

	// Team and alliance
	protected enum Team {
		RED,
		BLUE,
		NULL
	}
	protected Team team = Team.NULL;
	// Starting pose - override in child classes
	protected Pose startPose = new Pose(0, 0, 0);
	CustomTelemetry telemetryC;
	static DashboardTelemetryManager telemetryM;

	// State tracking for OpMode
	protected boolean autoSequenceStarted = false;

	@Override
	public void init() {
		// Initialize timers
		runtime = new ElapsedTime();
		autoTimer = new ElapsedTime();

		// Initialize telemetry
		telemetry.addData("Status", "Initializing...");
		telemetry.update();

		// Initialize Pedro Pathing follower
		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(startPose);

		// Initialize hub bulk caching
		HubInitializer.initBulkCaching(hardwareMap);
		telemetryM = DashboardTelemetryManager.create();
		telemetryC = new CustomTelemetry(telemetry, telemetryM);

		// Initialize AprilTag localizer
		try {
			aprilTagLocalizer = new AprilTagLocalizer();
			aprilTagLocalizer.initAprilTag(hardwareMap, "Webcam 1");
		} catch (Exception e) {
			telemetryC.addData("AprilTag Error", e.getMessage());
		}

		// Initialize servos
		try {
			elevationServo = hardwareMap.get(Servo.class, "elevationServo");
			azimuthServo0 = hardwareMap.get(Servo.class, "azimuthServo0");
			azimuthServo1 = hardwareMap.get(Servo.class, "azimuthServo1");
		} catch (Exception e) {
			telemetryC.addData("Servo Init Error", e.getMessage());
		}

		// Initialize launcher motor
		try {
			launcher = new CustomMotor(
				hardwareMap,
				"launcher",
				true,
				1425.1,
				new CustomPIDFController(0.1, 0.01, 0.005, 0.0)
			);
		} catch (Exception e) {
			telemetryC.addData("Launcher Init Error", e.getMessage());
		}

		// Initialize color sensor
		try {
			colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
		} catch (Exception e) {
			telemetryC.addData("Color Sensor Init Error", e.getMessage());
		}

		// Initial team selection prompt
		telemetryC.addData("Team", "Press A for RED, B for BLUE");
		telemetryC.addData("Current Team", team.toString());
		telemetryC.update();
	}

	@Override
	public void init_loop() {
		// Allow team selection during init
		if (gamepad1.a) {
			team = Team.RED;
		} else if (gamepad1.b) {
			team = Team.BLUE;
		}

		telemetryC.addData("Selected Team", team.toString());
		if (team == Team.NULL) {
			telemetryC.addData("Warning", "No team selected!");
		}
		telemetryC.addData("Status", "Ready to start");

		telemetryC.update();
	}

	@Override
	public void start() {
		runtime.reset();
		autoTimer.reset();
		autoSequenceStarted = false;
	}

	@Override
	public void loop() {
		// Update follower each loop iteration
		updateFollower();

		// Track state changes
		boolean stateChanged = (currentState != previousState);
		previousState = currentState;

		// State machine - override handleState() in child classes
		handleState(stateChanged);

		// Update telemetry with state info
		telemetryC.addData("Current State", currentState.toString());
		telemetryC.addData("Runtime", runtime.seconds());
		telemetryC.update();
	}

	@Override
	public void stop() {
		// Transfer data to TeleOp
		AutoToTeleDataTransferer dataTransfer = AutoToTeleDataTransferer.getInstance();

		// Store final pose
		if (follower != null) {
			Pose finalPose = follower.getPose();
			dataTransfer.setEndPose(finalPose);
			follower.breakFollowing();
		}

		// Store alliance color
		String allianceColor = (team == Team.RED) ? "RED" : (team == Team.BLUE) ? "BLUE" : "UNKNOWN";
		dataTransfer.setAllianceColor(allianceColor);

		// Store whether auto completed successfully
		dataTransfer.setAutoCompleted(currentState == AutoState.FINISHED);

		// Store runtime
		if (autoTimer != null) {
			dataTransfer.setAutoRuntime(autoTimer.seconds());
		}

		// Store starting position (can be overridden in child classes)
		dataTransfer.setStartingPosition(startPose.getX() + "," + startPose.getY());

		// Add completion notes
		if (currentState == AutoState.FINISHED) {
			dataTransfer.setAutoNotes("Autonomous completed successfully");
		} else {
			dataTransfer.setAutoNotes("Autonomous stopped at state: " + currentState.toString());
		}

		// Log summary to telemetry
		telemetryC.addData("Transfer Summary", dataTransfer.getSummary());
		telemetryC.update();
	}

	/**
	 * Override this method in child classes to handle state machine logic.
	 * Use switch statement on currentState to execute different actions.
	 * @param stateChanged true if the state just changed this loop iteration
	 */
	protected void handleState(boolean stateChanged) {
	switch (team) {
			case RED:
				handleRedState(stateChanged);
				break;
			case BLUE:
				handleBlueState(stateChanged);
				break;
			default:
				// No team selected - do nothing
				telemetryC.addData("Error", "No team selected!");
				break;

		}
	}

	/**
	 * Handle state machine for RED team autonomous
	 * Override in child classes for custom red auto
	 */
	protected void handleRedState(boolean stateChanged) {
		switch (currentState) {
			case IDLE:
				// Transition to first path/action
				setState(AutoState.PATH_1);
				break;

			case PATH_1:
				if (stateChanged) {
					// Start RED path here when entering state
					// follower.followPath(redPath1);
				}
				if (isPathComplete()) {
					setState(AutoState.ACTION_1);
				}
				break;

			case ACTION_1:
				if (stateChanged) {
					// Perform RED action when entering state
					startDelay(500); // Example: wait 500ms
				}
				if (isDelayComplete()) {
					setState(AutoState.FINISHED);
				}
				break;

			case FINISHED:
				// Autonomous complete
				break;

			default:
				break;
		}
	}

	/**
	 * Handle state machine for BLUE team autonomous
	 * Override in child classes for custom blue auto
	 */
	protected void handleBlueState(boolean stateChanged) {
		switch (currentState) {
			case IDLE:
				// Transition to first path/action
				setState(AutoState.PATH_1);
				break;

			case PATH_1:
				if (stateChanged) {
					// Start BLUE path here when entering state
					// follower.followPath(bluePath1);
				}
				if (isPathComplete()) {
					setState(AutoState.ACTION_1);
				}
				break;

			case ACTION_1:
				if (stateChanged) {
					// Perform BLUE action when entering state
					startDelay(500); // Example: wait 500ms
				}
				if (isDelayComplete()) {
					setState(AutoState.FINISHED);
				}
				break;

			case FINISHED:
				// Autonomous complete
				break;

			default:
				break;
		}
	}

	/**
	 * Set the current state (use this to transition between states)
	 */
	protected void setState(AutoState newState) {
		currentState = newState;
	}

	/**
	 * Check if currently in a specific state
	 */
	protected boolean isInState(AutoState state) {
		return currentState == state;
	}

	/**
	 * Get the next state after a WAIT state (override in child if using WAIT)
	 */
	protected AutoState nextStateAfterWait = AutoState.FINISHED;

	/**
	 * Start a wait state with a specified duration and next state
	 */
	protected void startWait(long milliseconds, AutoState nextState) {
		startDelay(milliseconds);
		nextStateAfterWait = nextState;
		setState(AutoState.WAIT);
	}

	/**
	 * Helper method to update follower (call in loop)
	 */
	protected void updateFollower() {
		if (follower != null) {
			follower.update();
		}
	}

	/**
	 * Helper method to check if follower is busy
	 */
	protected boolean isBusy() {
		return follower != null && follower.isBusy();
	}

	/**
	 * Helper method to check if path is complete and update telemetry
	 * Note: In iterative OpMode, use this in loop() with state machine pattern
	 * instead of blocking waits
	 */
	protected boolean isPathComplete() {
		if (isBusy()) {
			telemetryC.addData("Status", "Following path...");
			telemetryC.addData("X", follower.getPose().getX());
			telemetryC.addData("Y", follower.getPose().getY());
			telemetryC.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
			telemetryC.update();
			return false;
		}
		return true;
	}

	/**
	 * Helper method to set servo position
	 */
	protected void setServoPosition(Servo servo, double position) {
		if (servo != null) {
			servo.setPosition(position);
		}
	}

	/**
	 * Helper timer for non-blocking delays
	 */
	protected ElapsedTime delayTimer = new ElapsedTime();
	protected long delayTarget = 0;

	/**
	 * Start a delay timer (use with isDelayComplete() for non-blocking delays)
	 */
	protected void startDelay(long milliseconds) {
		delayTimer.reset();
		delayTarget = milliseconds;
	}

	/**
	 * Check if the delay started with startDelay() is complete
	 */
	protected boolean isDelayComplete() {
		return delayTimer.milliseconds() >= delayTarget;
	}
}
