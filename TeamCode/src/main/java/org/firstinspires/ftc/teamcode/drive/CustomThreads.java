package org.firstinspires.ftc.teamcode.drive;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.CpuMonitor;

public class CustomThreads {

    RobotCoreCustom robotCoreCustom;
    Follower follower;
    private Thread drawingThread;
    private volatile boolean drawingThreadRunning = false;

    private Thread CPUThread;
    private volatile boolean CPUThreadRunning = false;
    private double cpuUsage = 0.0;
    private double cpuTemp = 0.0;

    private Thread azimuthPIDThread;
    private volatile boolean azimuthPIDThreadRunning = false;
    private RobotCoreCustom.CustomAxonServoController azimuthServo;


    public CustomThreads(RobotCoreCustom robotCoreCustom, Follower follower) {
        this.robotCoreCustom = robotCoreCustom;
        this.follower = follower;
    }

    /**
     * Sets the azimuth servo controller for the PID thread
     * @param azimuthServo The CustomAxonServoController to run PID on
     */
    public void setAzimuthServo(RobotCoreCustom.CustomAxonServoController azimuthServo) {
        this.azimuthServo = azimuthServo;
    }
    public void startDrawingThread() {
        // Prevent starting multiple threads
        if (drawingThreadRunning) {
            return;
        }

        drawingThreadRunning = true;
        drawingThread = new Thread(() -> {
            while (drawingThreadRunning) {
                try {
                    robotCoreCustom.drawCurrentAndHistory(follower);
                    Thread.sleep(10); // Adjust the sleep time as needed
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception e) {
                    // Log error but don't crash the thread
                    System.err.println("Error in drawing thread: " + e.getMessage());
                }
            }
        });
        drawingThread.start();
    }
    public void stopDrawingThread() {
        drawingThreadRunning = false;
        if (drawingThread != null) {
            try {
                drawingThread.join(100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
    public void startCPUMonThread() {
        // Prevent starting multiple threads
        if (CPUThreadRunning) {
            return;
        }

        CPUThreadRunning = true;
        CpuMonitor cpuMonitor = new CpuMonitor();
        CPUThread = new Thread(() -> {
            while (CPUThreadRunning) {
                try {
                    cpuUsage = cpuMonitor.getCpuUsage();
                    cpuTemp = cpuMonitor.getCpuTemp();
                    Thread.sleep(1000); // Update every second
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception e) {
                    // Log error but don't crash the thread
                    System.err.println("Error in CPU monitor thread: " + e.getMessage());
                }
            }
        });
        CPUThread.start();
    }
    public void stopCPUMonThread() {
        CPUThreadRunning = false;
        if (CPUThread != null) {
            try {
                CPUThread.join(100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
    public double getCpuUsage() {
        return cpuUsage;
    }
    public double getCpuTemp() {
        return cpuTemp;
    }

    /**
     * Starts the azimuth servo PID control thread
     * Must call setAzimuthServo() before starting this thread
     */
    public void startAzimuthPIDThread() {
        if (azimuthServo == null) {
            throw new IllegalStateException("Azimuth servo controller must be set before starting PID thread. Call setAzimuthServo() first.");
        }

        // Prevent starting multiple threads
        if (azimuthPIDThreadRunning) {
            return;
        }

        azimuthPIDThreadRunning = true;
        azimuthPIDThread = new Thread(() -> {
            while (azimuthPIDThreadRunning) {
                try {
                    // Only run PID loop if turret is enabled in MDOConstants
                    if (MDOConstants.EnableTurret) {
                        azimuthServo.servoPidLoop();
                    } else {
                        azimuthServo.stopServo();
                    }
                    Thread.sleep(5); // Run PID loop every 5ms for responsive control
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception e) {
                    // Log error but don't crash the thread
                    System.err.println("Error in azimuth PID loop: " + e.getMessage());
                }
            }
        });
        azimuthPIDThread.start();
    }

    /**
     * Stops the azimuth servo PID control thread
     */
    public void stopAzimuthPIDThread() {
        azimuthPIDThreadRunning = false;
        if (azimuthPIDThread != null) {
            try {
                azimuthPIDThread.join(100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
}
