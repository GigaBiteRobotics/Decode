package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.DashboardTelemetryManager;

/**
 * Thread-safe telemetry wrapper that buffers addData calls from any thread
 * and flushes them in insertion order when update() is called.
 *
 * Usage: call addData() from any thread at any time. Call update() from the
 * main loop thread to push all buffered entries to the driver station / dashboard.
 */
public class CustomTelemetry {
	private final Telemetry combined;
	private final java.util.concurrent.ConcurrentLinkedQueue<String[]> buffer =
			new java.util.concurrent.ConcurrentLinkedQueue<>();

	public CustomTelemetry(Telemetry telemetry, DashboardTelemetryManager ignored) {
		this.combined = new com.acmerobotics.dashboard.telemetry.MultipleTelemetry(
				telemetry, com.acmerobotics.dashboard.FtcDashboard.getInstance().getTelemetry());
	}

	/**
	 * Thread-safe: buffers a telemetry entry. Can be called from any thread.
	 */
	public void addData(String caption, Object value) {
		buffer.add(new String[]{caption, String.valueOf(value)});
	}

	/**
	 * Drains all buffered entries in insertion order to the underlying telemetry,
	 * then pushes the update. Should be called from the main loop thread.
	 */
	public void update() {
		String[] entry;
		while ((entry = buffer.poll()) != null) {
			combined.addData(entry[0], entry[1]);
		}
		combined.update();
	}
}

