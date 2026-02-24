package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * Drop-in replacement for Panels TelemetryManager.
 * Routes telemetry to both the Driver Station and FTC Dashboard.
 * Preserves key-value structure so FTC Dashboard can graph numeric values.
 */
public class DashboardTelemetryManager {
    private final List<Object[]> entries = new ArrayList<>();
    // null = plain line, Object[2] = {key, value} pair

    public DashboardTelemetryManager() {}

    /**
     * Add a debug line (plain text, not graphable).
     */
    public void debug(String message) {
        entries.add(new Object[]{null, message});
    }

    /**
     * Add a key-value pair (graphable in FTC Dashboard if value is numeric).
     */
    public void debug(String key, Object value) {
        entries.add(new Object[]{key, value});
    }

    /**
     * Flush all entries to telemetry (both Driver Station and FTC Dashboard).
     * Key-value pairs are sent via addData() so Dashboard can graph them.
     */
    public void update(Telemetry telemetry) {
        Telemetry dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        flushTo(dashTelemetry);
    }

    /**
     * Flush all entries (no-arg version, sends to FTC Dashboard only).
     */
    public void update() {
        Telemetry dashTelemetry = FtcDashboard.getInstance().getTelemetry();
        flushTo(dashTelemetry);
    }

    private void flushTo(Telemetry target) {
        for (Object[] entry : entries) {
            if (entry[0] == null) {
                // Plain text line
                target.addLine((String) entry[1]);
            } else {
                // Structured key-value pair — Dashboard can graph numeric values
                target.addData((String) entry[0], entry[1]);
            }
        }
        target.update();
        entries.clear();
    }

    /**
     * Static factory matching the old PanelsTelemetry.INSTANCE.getTelemetry() pattern.
     */
    public static DashboardTelemetryManager create() {
        return new DashboardTelemetryManager();
    }
}



