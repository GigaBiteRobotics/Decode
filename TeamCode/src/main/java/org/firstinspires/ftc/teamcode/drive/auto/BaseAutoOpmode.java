package org.firstinspires.ftc.teamcode.drive.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.drive.CustomPIDFController;
import org.firstinspires.ftc.teamcode.drive.RobotCoreCustom;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BaseAuto(DO NOT USE)", group = "z")
public class BaseAutoOpmode extends LinearOpMode {

	// Robot core components
	protected Follower follower;
	protected RobotCoreCustom robotCoreCustom;
	protected AprilTagLocalizer aprilTagLocalizer;

	// Servos
	protected Servo elevationServo, azimuthServo0, azimuthServo1;

	// Motors
	protected RobotCoreCustom.CustomMotor launcher;

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
	RobotCoreCustom.CustomTelemetry telemetryC;
	static TelemetryManager telemetryM;

	@Override
	public void runOpMode() {
		// Initialize timers
		runtime = new ElapsedTime();
		autoTimer = new ElapsedTime();

		// Initialize telemetry
		telemetry.addData("Status", "Initializing...");
		telemetry.update();

		// Initialize Pedro Pathing follower
		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(startPose);

		// Initialize robot core
		robotCoreCustom = new RobotCoreCustom(hardwareMap, follower);
		telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
		telemetryC = new RobotCoreCustom.CustomTelemetry(telemetry, telemetryM);

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
			launcher = new RobotCoreCustom.CustomMotor(
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

		// Allow team selection during init
		telemetryC.addData("Team", "Press A for RED, B for BLUE");
		telemetryC.addData("Current Team", team.toString());
		telemetryC.update();

		while (!isStarted() && !isStopRequested()) {
			if (gamepad1.a) {
				team = Team.RED;
			} else if (gamepad1.b) {
				team = Team.BLUE;
			} else {
				if (team != Team.RED && team != Team.BLUE) {
					team = Team.NULL;
				}
			}

			telemetryC.addData("Selected Team", team.toString());
			if (team == Team.NULL) {
				telemetryC.addData("Warning", "No team selected!");
			}
			telemetryC.addData("Status", "Ready to start");

			telemetryC.update();
		}

		// Wait for start
		waitForStart();
		runtime.reset();
		autoTimer.reset();

		// Run the autonomous routine (to be implemented in child classes)
		if (opModeIsActive()) {
			runAutoSequence();
		}

		// Clean up
		if (follower != null) {
			follower.breakFollowing();
		}
	}

	/**
	 * Override this method in child classes to implement autonomous routine
	 */
	protected void runAutoSequence() {
		// Default implementation - does nothing
		telemetryC.addData("Warning", "runAutoSequence() not implemented");
		telemetryC.update();
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
	 * Helper method to wait for path to complete
	 */
	protected void waitForPath() {
		while (opModeIsActive() && isBusy()) {
			updateFollower();
			telemetryC.addData("Status", "Following path...");
			telemetryC.addData("X", follower.getPose().getX());
			telemetryC.addData("Y", follower.getPose().getY());
			telemetryC.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
			telemetryC.update();
		}
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
	 * Helper method to sleep safely
	 */
	protected void safeSleep(long milliseconds) {
		ElapsedTime timer = new ElapsedTime();
		while (opModeIsActive() && timer.milliseconds() < milliseconds) {
			updateFollower();
			idle();
		}
	}
}
