package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.MDOConstants;
import org.firstinspires.ftc.teamcode.modules.CustomTelemetry;
import org.firstinspires.ftc.teamcode.modules.CustomThreads;
import org.firstinspires.ftc.teamcode.modules.HubInitializer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.DashboardTelemetryManager;

@TeleOp(name = "Drive", group = "!advanced")
public class MainDriveOpmode extends OpMode {

	private Follower follower;
	private CustomTelemetry telemetryC;
	private CustomThreads customThreads;
	
	private int telemetryLoopCounter = 0;
	private static final int TELEMETRY_UPDATE_INTERVAL = 10;

	@Override
	public void init() {
		telemetryC = new CustomTelemetry(telemetry, DashboardTelemetryManager.create());
		HubInitializer.initBulkCaching(hardwareMap);

		// ONLY initialize follower (needed for drive)
		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(new Pose());
		follower.startTeleopDrive();

		customThreads = new CustomThreads(follower);
	}

	@Override
	public void start() {
		follower.startTeleopDrive();
	}

	@Override
	public void stop() {
		// Nothing to stop — no subsystems or threads
	}

	@SuppressLint("DefaultLocale")
	@Override
	public void loop() {
		// CRITICAL: ONLY DRIVE — no subsystem updates, no background threads
		// Everything except drive is DISABLED to prove where the latency comes from
		
		// 1. Bulk cache clear + drive update (REQUIRED for responsive drivetrain)
		HubInitializer.clearBulkCache();
		follower.setTeleOpDrive(
				-gamepad1.left_stick_y,
				-gamepad1.left_stick_x,
				-gamepad1.right_stick_x,
				MDOConstants.EnableFieldCentricDrive
		);
		follower.update();

		// 2. Minimal telemetry (every 10 loops to reduce WiFi payload)
		telemetryLoopCounter++;
		if (telemetryLoopCounter >= TELEMETRY_UPDATE_INTERVAL) {
			telemetryLoopCounter = 0;
			
			Pose currentPose = follower.getPose();
			if (currentPose == null) currentPose = new Pose(0, 0, 0);
			
			telemetryC.addData("Pose", String.format("(%.1f, %.1f, %.1f deg)",
					currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading())));
			telemetryC.addData("Status", "DRIVE ONLY - No Subsystems");
			telemetryC.update();
		}
	}
}