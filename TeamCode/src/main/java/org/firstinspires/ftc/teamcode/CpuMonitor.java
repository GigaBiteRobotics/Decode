package org.firstinspires.ftc.teamcode;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

public class CpuMonitor {
    private long prevIdle = 0;
    private long prevTotal = 0;

    public double getCpuUsage() {
        try {
            BufferedReader reader = new BufferedReader(new FileReader("/proc/stat"));
            String line = reader.readLine();
            reader.close();

            // Parse the first line: cpu  user nice system idle iowait irq softirq
            String[] tokens = line.split("\\s+");

            long idle = Long.parseLong(tokens[4]);
            long total = 0;
            for (int i = 1; i < tokens.length; i++) {
                total += Long.parseLong(tokens[i]);
            }

            long diffIdle = idle - prevIdle;
            long diffTotal = total - prevTotal;

            prevIdle = idle;
            prevTotal = total;

            if (diffTotal == 0) return 0;

            return (1.0 - (double)diffIdle / diffTotal) * 100.0;

        } catch (Exception e) {
            return -1;
        }
    }
    public double getCpuTemp() {
        try {
            // Try common thermal zone locations
            String[] thermalPaths = {
                    "/sys/class/thermal/thermal_zone0/temp",
                    "/sys/class/thermal/thermal_zone1/temp",
                    "/sys/devices/virtual/thermal/thermal_zone0/temp"
            };

            for (String path : thermalPaths) {
                File tempFile = new File(path);
                if (tempFile.exists()) {
                    BufferedReader reader = new BufferedReader(new FileReader(tempFile));
                    String temp = reader.readLine();
                    reader.close();

                    // Temperature is usually in millidegrees Celsius
                    return Double.parseDouble(temp) / 1000.0;
                }
            }
        } catch (Exception e) {
            return -1;
        }
        return -1;
    }
}