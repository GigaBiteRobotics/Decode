package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.constants.MDOConstants;
import org.firstinspires.ftc.teamcode.modules.CustomMotorController;
import org.firstinspires.ftc.teamcode.modules.CustomTelemetry;
import org.firstinspires.ftc.teamcode.modules.GamepadEventHandler;
import org.firstinspires.ftc.teamcode.modules.HubInitializer;
import org.firstinspires.ftc.teamcode.modules.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.DashboardTelemetryManager;

/**
 * MINIMAL drive opmode with ONE subsystem (IntakeSubsystem) for testing.
 * Use this to isolate which subsystem is blocking others and causing latency issues.
 * 
 * Test sequence:
 * 1. Run MinimalDrive (no subsystems) - baseline latency
 * 2. Run MinimalDriveWithIntake (current) - test intake isolation
 * 3. Swap subsystems as needed to identify culprits
 */
@TeleOp(name = "MinimalDriveWithIntake", group = "!advanced")
public class MinimalDriveOpmode extends OpMode {

	private Follower follower;
	private CustomTelemetry telemetryC;
	private IntakeSubsystem intake;
	private DistanceSensor ballDistSensor;
	private GamepadEventHandler gp2Handler;
	private int loopCounter = 0;

	@Override
	public void init() {
		telemetryC = new CustomTelemetry(telemetry, DashboardTelemetryManager.create());
		HubInitializer.initBulkCaching(hardwareMap);

		// Initialize follower (drive)
		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(new Pose());
		follower.startTeleopDrive();

		// ===== INTAKE SUBSYSTEM INITIALIZATION =====
		try {
			CustomMotorController intakeMotor = new CustomMotorController(
					hardwareMap,
					new String[]{"intake"},
					new boolean[]{true},
					false,
					28.0,
					new PIDFController(0, 0, 0, 0)
			);
			ballDistSensor = hardwareMap.get(DistanceSensor.class, "frontRange");
			intake = new IntakeSubsystem(intakeMotor, ballDistSensor);

			// Setup gamepad controls for intake
			gp2Handler = new GamepadEventHandler();
			gp2Handler.onPress("intakeIn", gp -> gp.a, () -> intake.toggleIn());
			gp2Handler.onPress("intakeOut", gp -> gp.b, () -> intake.toggleOut());

			telemetryC.addData("Intake Init", "SUCCESS");
		} catch (Exception e) {
			telemetryC.addData("Intake Init Error", e.getMessage());
		}

		telemetryC.update();
	}

	@Override
	public void start() {
		follower.startTeleopDrive();
	}

	@Override
	public void stop() {
		// Nothing to stop
	}

	@SuppressLint("DefaultLocale")
	@Override
	public void loop() {
		// ===== TIMING MEASUREMENT =====
		long loopStartTime = System.currentTimeMillis();

		// ===== DRIVE CONTROL =====
		HubInitializer.clearBulkCache();
		long driveStartTime = System.currentTimeMillis();

		follower.setTeleOpDrive(
				-gamepad1.left_stick_y,
				-gamepad1.left_stick_x,
				-gamepad1.right_stick_x,
				MDOConstants.EnableFieldCentricDrive
		);
		follower.update();
		long driveTime = System.currentTimeMillis() - driveStartTime;

		// ===== INTAKE CONTROL =====
		long intakeStartTime = System.currentTimeMillis();

		// Update gamepad inputs
		if (gp2Handler != null) {
			gp2Handler.update(gamepad2);
		}

		// Update intake (dummy values for now - no background threads)
		if (intake != null) {
			intake.update(0, MDOConstants.BallDetectionDistanceCm + 10);  // 0 balls, distance out of range
		}
		long intakeTime = System.currentTimeMillis() - intakeStartTime;

		// ===== TELEMETRY - Every 10 loops =====
		loopCounter++;
		if (loopCounter >= 10) {
			loopCounter = 0;
			long totalLoopTime = System.currentTimeMillis() - loopStartTime;

			Pose pose = follower.getPose();
			if (pose == null) pose = new Pose(0, 0, 0);

			telemetryC.addData("--- MINIMAL DRIVE WITH INTAKE ---", "");
			telemetryC.addData("Pose", String.format("(%.1f, %.1f, %.1f deg)",
					pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading())));
			telemetryC.addData("Intake State", intake != null ? intake.getState() : "NULL");
			telemetryC.addData("Intake Distance (cm)", intake != null ? String.format("%.1f", intake.GetDistanceBallSensorCM()) : "N/A");

			telemetryC.addData("--- TIMING (ms) ---", "");
			telemetryC.addData("Drive Control", String.format("%.1f", (double) driveTime));
			telemetryC.addData("Intake Control", String.format("%.1f", (double) intakeTime));
			telemetryC.addData("Total Loop", String.format("%.1f", (double) totalLoopTime));

			telemetryC.update();
		}
	}
}
