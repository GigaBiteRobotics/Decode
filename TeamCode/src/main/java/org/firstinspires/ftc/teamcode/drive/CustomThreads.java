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

    private Thread sorterThread;
    private volatile boolean sorterThreadRunning = false;
    private RobotCoreCustom.CustomSorterController sorterController;

    private Thread launcherPIDThread;
    private volatile boolean launcherPIDThreadRunning = false;
    private RobotCoreCustom.CustomMotorController launcherMotors;

    private Thread driveThread;
    private volatile boolean driveThreadRunning = false;
    private volatile double driveY = 0.0;
    private volatile double driveX = 0.0;
    private volatile double driveRotation = 0.0;
    private volatile boolean driveFieldCentric = true;

    private Thread aprilTagThread;
    private volatile boolean aprilTagThreadRunning = false;
    private volatile Double[] cachedAprilPose = null;
    private AprilTagLocalizer aprilTagLocalizer;

    private Thread followerUpdateThread;
    private volatile boolean followerUpdateThreadRunning = false;


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

    /**
     * Sets the sorter controller to be managed by the thread
     * @param sorterController The CustomSorterController
     */
    public void setSorterController(RobotCoreCustom.CustomSorterController sorterController) {
        this.sorterController = sorterController;
    }

    /**
     * Sets the launcher motor controller for the PID thread
     * @param launcherMotors The CustomMotorController to run PID on
     */
    public void setLauncherMotors(RobotCoreCustom.CustomMotorController launcherMotors) {
        this.launcherMotors = launcherMotors;
    }

    /**
     * Sets the AprilTag localizer for the AprilTag thread
     * @param localizer The AprilTagLocalizer to process in background
     */
    public void setAprilTagLocalizer(AprilTagLocalizer localizer) {
        this.aprilTagLocalizer = localizer;
    }

    /**
     * Gets the cached AprilTag pose from the background thread
     * @return The cached pose array or null if no tag detected
     */
    public Double[] getCachedAprilPose() {
        return cachedAprilPose;
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

    public void startSorterThread() {
        if (sorterThreadRunning || sorterController == null) {
            return;
        }

        sorterThreadRunning = true;
        sorterThread = new Thread(() -> {
            while (sorterThreadRunning) {
                try {
                    // This is the heavy I2C operation
                    sorterController.updateSensors();

                    // We can also run lifter updater here if desired,
                    // but it's lightweight (servos) so main thread is fine too.

                    // Sleep to yield and prevent CPU hogging,
                    // but small enough to be responsive
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception e) {
                    System.err.println("Error in sorter thread: " + e.getMessage());
                }
            }
        });
        sorterThread.start();
    }

    public void stopSorterThread() {
        sorterThreadRunning = false;
        if (sorterThread != null) {
            try {
                sorterThread.join(100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    public void startLauncherPIDThread() {
        if (launcherMotors == null) {
            throw new IllegalStateException("Launcher motors must be set before starting PID thread. Call setLauncherMotors() first.");
        }

        if (launcherPIDThreadRunning) {
            return;
        }

        launcherPIDThreadRunning = true;
        launcherPIDThread = new Thread(() -> {
            while (launcherPIDThreadRunning) {
                try {
                    // Update PID for launcher motors (reads encoders, calculates power, sets power)
                    // This offloads hardware I/O and control math from the main loop
                    launcherMotors.updateRPMPID();

                    // Run at ~50Hz (20ms)
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception e) {
                    System.err.println("Error in launcher PID thread: " + e.getMessage());
                }
            }
        });
        launcherPIDThread.start();
    }

    public void stopLauncherPIDThread() {
        launcherPIDThreadRunning = false;
        if (launcherPIDThread != null) {
            try {
                launcherPIDThread.join(100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    /**
     * Sets the drive inputs for the drive thread to process
     * @param y Forward/backward input (-1 to 1)
     * @param x Strafe left/right input (-1 to 1)
     * @param rotation Rotation input (-1 to 1)
     * @param fieldCentric Whether to use field-centric drive
     */
    public void setDriveInputs(double y, double x, double rotation, boolean fieldCentric) {
        this.driveY = y;
        this.driveX = x;
        this.driveRotation = rotation;
        this.driveFieldCentric = fieldCentric;
    }

    /**
     * Starts the drive control thread
     * Processes gamepad inputs and updates follower at high frequency
     */
    public void startDriveThread() {
        if (driveThreadRunning) {
            return;
        }

        driveThreadRunning = true;
        driveThread = new Thread(() -> {
            while (driveThreadRunning) {
                try {
                    // Update follower with current drive inputs
                    follower.setTeleOpDrive(driveY, driveX, driveRotation, driveFieldCentric);

                    // Run at high frequency for responsive driving (~200Hz)
                    Thread.sleep(5);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception e) {
                    System.err.println("Error in drive thread: " + e.getMessage());
                }
            }
        });
        driveThread.start();
    }

    /**
     * Stops the drive control thread
     */
    public void stopDriveThread() {
        driveThreadRunning = false;
        if (driveThread != null) {
            try {
                driveThread.join(100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    /**
     * Starts the AprilTag processing thread
     * Processes AprilTag detections in background at configurable frequency
     */
    public void startAprilTagThread() {
        if (aprilTagThreadRunning || aprilTagLocalizer == null) {
            return;
        }

        aprilTagThreadRunning = true;
        aprilTagThread = new Thread(() -> {
            while (aprilTagThreadRunning) {
                try {
                    // Get AprilTag position (this is the expensive operation)
                    cachedAprilPose = aprilTagLocalizer.getPosition();

                    // Sleep based on configured update interval
                    // Higher interval = less CPU usage but less frequent updates
                    Thread.sleep(MDOConstants.AprilTagUpdateIntervalMs);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception e) {
                    System.err.println("Error in AprilTag thread: " + e.getMessage());
                }
            }
        });
        aprilTagThread.start();
    }

    /**
     * Stops the AprilTag processing thread
     */
    public void stopAprilTagThread() {
        aprilTagThreadRunning = false;
        if (aprilTagThread != null) {
            try {
                aprilTagThread.join(100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    /**
     * Starts the follower update thread
     * This offloads follower.update() to run continuously in background
     * This is the KEY to improving loop performance - follower.update() is expensive
     */
    public void startFollowerUpdateThread() {
        if (followerUpdateThreadRunning) {
            return;
        }

        followerUpdateThreadRunning = true;
        followerUpdateThread = new Thread(() -> {
            while (followerUpdateThreadRunning) {
                try {
                    // This is the expensive operation that updates odometry, path following, etc.
                    follower.update();

                    // Run at high frequency for accurate odometry (~200Hz)
                    Thread.sleep(5);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception e) {
                    System.err.println("Error in follower update thread: " + e.getMessage());
                }
            }
        });
        followerUpdateThread.start();
    }

    /**
     * Stops the follower update thread
     */
    public void stopFollowerUpdateThread() {
        followerUpdateThreadRunning = false;
        if (followerUpdateThread != null) {
            try {
                followerUpdateThread.join(100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
}
