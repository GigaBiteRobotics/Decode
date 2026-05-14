package org.firstinspires.ftc.teamcode.modules;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.CpuMonitor;
import org.firstinspires.ftc.teamcode.constants.MDOConstants;
import org.firstinspires.ftc.teamcode.drive.AprilTagLocalizer;

public class CustomThreads {

	Follower follower;

	// Thread-safe pose caching to prevent race conditions when accessing follower.getPose()
	private final Object followerLock = new Object();
	private volatile Pose cachedPose = null;

	private Thread drawingThread;
	private volatile boolean drawingThreadRunning = false;

	private Thread CPUThread;
	private volatile boolean CPUThreadRunning = false;
	private double cpuUsage = 0.0;
	private double cpuTemp = 0.0;

	private Thread azimuthPIDThread;
	private volatile boolean azimuthPIDThreadRunning = false;
	private CustomServoController azimuthServo;

	private Thread sorterThread;
	private volatile boolean sorterThreadRunning = false;
	private CustomSorterController sorterController;

	private Thread launcherPIDThread;
	private volatile boolean launcherPIDThreadRunning = false;
	private CustomMotorController launcherMotors;

	// Live gamepad reference — set once in start(). The follower update thread
	// reads stick values directly at ~500 Hz, fresher than the main loop (~50 Hz).
	private volatile Gamepad liveGamepad = null;

	// When true, the follower update thread skips setTeleOpDrive() so it cannot
	// interfere with an active path follower.
	private volatile boolean driveAutomatedActive = false;

	// Field-centric flag updated by the main loop.
	private volatile boolean driveFieldCentric = true;

	private Thread aprilTagThread;
	private volatile boolean aprilTagThreadRunning = false;
	private volatile Double[] cachedAprilPose = null;
	private AprilTagLocalizer aprilTagLocalizer;

	private Thread followerUpdateThread;
	private volatile boolean followerUpdateThreadRunning = false;
	/** Cached result of follower.isBusy() updated after each follower.update(). Lock-free read for main loop. */
	private volatile boolean cachedFollowerBusy = false;

	// ===== BALL DISTANCE SENSOR THREAD =====
	// Moves the blocking I2C read off the main loop into a background thread so
	// bulk-read caching is not disrupted and the main loop stays non-blocking.
	private Thread ballDistanceSensorThread;
	private volatile boolean ballDistanceSensorThreadRunning = false;
	private com.qualcomm.robotcore.hardware.DistanceSensor ballDistanceSensor;

	/** Latest filtered distance reading in centimetres. Initialised to max so no false "ball present" on startup. */
	private volatile double cachedBallDistanceCm = 9999.0;

	/** True when the sensor has returned consecutive out-of-range values and may be stuck. */
	private volatile boolean ballSensorStuck = false;


	public CustomThreads(Follower follower) {
		this.follower = follower;
		// Initialize cached pose to default - will be updated by followerUpdateThread
		// We don't call follower.getPose() here because it might not be safe during construction
		this.cachedPose = new Pose(0, 0, 0);
	}

	/**
	 * Thread-safe method to get the current robot pose.
	 * This method returns the cached pose that is updated by the follower update thread.
	 * IMPORTANT: This method NEVER calls follower.getPose() directly to prevent race conditions.
	 * The cache is updated ONLY by the followerUpdateThread after each follower.update() call.
	 *
	 * @return A copy of the current cached pose (thread-safe)
	 */
	public Pose getThreadSafePose() {
		synchronized (followerLock) {
			if (cachedPose == null) {
				return new Pose(0, 0, 0);
			}
			return new Pose(cachedPose.getX(), cachedPose.getY(), cachedPose.getHeading());
		}
	}

	/**
	 * Thread-safe method to set the follower pose.
	 * Use this when updating pose from AprilTag or other sources while threaded update is running.
	 *
	 * @param pose The new pose to set
	 */
	public void setThreadSafePose(Pose pose) {
		synchronized (followerLock) {
			try {
				if (follower != null && pose != null) {
					follower.setPose(pose);
					// Also update cached pose immediately
					cachedPose = new Pose(pose.getX(), pose.getY(), pose.getHeading());
				}
			} catch (Exception e) {
				System.err.println("Error setting pose: " + e.getMessage());
			}
		}
	}

	/**
	 * Thread-safe method to update the cached pose.
	 * Call this after follower.update() to refresh the cache.
	 */
	public void updateCachedPose() {
		synchronized (followerLock) {
			try {
				if (follower != null) {
					Pose currentPose = follower.getPose();
					if (currentPose != null) {
						cachedPose = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
					}
				}
			} catch (Exception e) {
				System.err.println("Error updating cached pose: " + e.getMessage());
			}
		}
	}

	/**
	 * Sets the azimuth servo controller for the PID thread.
	 *
	 * @param azimuthServo The CustomServoController to run PID on
	 */
	public void setAzimuthServo(CustomServoController azimuthServo) {
		this.azimuthServo = azimuthServo;
	}

	/**
	 * Sets the sorter controller to be managed by the thread.
	 *
	 * @param sorterController The CustomSorterController
	 */
	public void setSorterController(CustomSorterController sorterController) {
		this.sorterController = sorterController;
	}

	/**
	 * Sets the launcher motor controller for the PID thread.
	 *
	 * @param launcherMotors The CustomMotorController to run PID on
	 */
	public void setLauncherMotors(CustomMotorController launcherMotors) {
		this.launcherMotors = launcherMotors;
	}

	/**
	 * Sets the AprilTag localizer for the AprilTag thread.
	 *
	 * @param localizer The AprilTagLocalizer to process in background
	 */
	public void setAprilTagLocalizer(AprilTagLocalizer localizer) {
		this.aprilTagLocalizer = localizer;
	}

	/**
	 * Gets the cached AprilTag pose from the background thread.
	 *
	 * @return The cached pose array or null if no tag detected
	 */
	public Double[] getCachedAprilPose() {
		return cachedAprilPose;
	}

	// ===== DISTANCE SENSOR API =====

	/**
	 * Sets the distance sensor to be read in the background thread.
	 * Must be called before {@link #startDistanceSensorThread()}.
	 */
	public void setDistanceSensor(com.qualcomm.robotcore.hardware.DistanceSensor sensor) {
		this.ballDistanceSensor = sensor;
	}

	/**
	 * Returns the latest cached distance reading in centimetres.
	 * Non-blocking — always returns the most recent value written by the sensor thread.
	 */
	public double getCachedBallDistanceCm() {
		return cachedBallDistanceCm;
	}

	/**
	 * Returns {@code true} while the sensor is returning consecutive out-of-range
	 * readings (i.e. it appears stuck at 800+). Cleared automatically when a valid
	 * reading arrives.
	 */
	public boolean isBallSensorStuck() {
		return ballSensorStuck;
	}

	// ===== LIVE GAMEPAD / AUTOMATED DRIVE API =====

	/**
	 * Provides a live Gamepad reference so the drive thread can read inputs
	 * directly at its own frequency instead of waiting for loop() to relay them.
	 * Call once in OpMode.start().
	 *
	 * @param gamepad The OpMode's gamepad1 instance
	 */
	public void setLiveGamepad(Gamepad gamepad) {
		this.liveGamepad = gamepad;
	}

	/**
	 * Tells the drive thread whether automated path following is active.
	 * When true the drive thread will not call setTeleOpDrive() so it cannot
	 * interfere with the follower's path controller.
	 *
	 * @param active true while a path is being followed or a position is held
	 */
	public void setDriveAutomatedActive(boolean active) {
		this.driveAutomatedActive = active;
	}

	// ===== THREAD-SAFE FOLLOWER CONTROL =====

	/**
	 * Thread-safe wrapper for follower.followPath().
	 * <p>
	 * Acquires the shared followerLock (the same lock used by the follower update thread)
	 * so this call never races with an in-progress follower.update().
	 * <p>
	 * driveAutomatedActive is set to true INSIDE the lock, before followPath() is called,
	 * so the drive thread is blocked from interfering before path following begins.
	 *
	 * @param chain   The PathChain to follow
	 * @param holdEnd Whether to hold position at the end of the path
	 */
	public void safeFollowPath(PathChain chain, boolean holdEnd) {
		synchronized (followerLock) {
			driveAutomatedActive = true; // suppress drive thread BEFORE path starts
			follower.followPath(chain, holdEnd);
		}
	}

	/**
	 * Thread-safe wrapper for follower.isBusy().
	 * Acquires followerLock to avoid a data race with follower.update().
	 */
	public boolean safeIsBusy() {
		if (followerUpdateThreadRunning) {
			// Lock-free fast path: return the value cached inside follower update thread.
			return cachedFollowerBusy;
		}
		synchronized (followerLock) {
			return follower.isBusy();
		}
	}

	/**
	 * Thread-safe wrapper for follower.setTeleOpDrive() for use from the main loop
	 * when the follower update thread is running. Acquires followerLock so it never
	 * races with follower.update().
	 */
	public void safeFollowerSetTeleOpDrive(double y, double x, double rotation, boolean fieldCentric) {
		synchronized (followerLock) {
			follower.setTeleOpDrive(y, x, rotation, fieldCentric);
		}
	}

	/**
	 * Thread-safe wrapper for follower.startTeleopDrive().
	 * <p>
	 * Acquires the shared followerLock so this call never races with follower.update().
	 * driveAutomatedActive is cleared to false AFTER the lock is released so the
	 * drive thread regains control only once teleop drive mode is fully active.
	 */
	public void safeStartTeleopDrive() {
		synchronized (followerLock) {
			follower.startTeleopDrive();
		}
		driveAutomatedActive = false; // restore drive thread control after teleop mode is set
	}

	// ===== THREAD METHODS =====

	public void startDrawingThread() {
		if (drawingThreadRunning) {
			return;
		}

		drawingThreadRunning = true;
		drawingThread = new Thread(() -> {
			while (drawingThreadRunning) {
				try {
					Pose safePose = getThreadSafePose();
					DashboardDrawing.drawCurrentWithPose(safePose);
					Thread.sleep(10);
				} catch (InterruptedException e) {
					Thread.currentThread().interrupt();
					break;
				} catch (Exception e) {
					System.err.println("Error in drawing thread: " + e.getMessage());
				}
			}
		});
		drawingThread.setDaemon(true);
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
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					Thread.currentThread().interrupt();
					break;
				} catch (Exception e) {
					System.err.println("Error in CPU monitor thread: " + e.getMessage());
				}
			}
		});
		CPUThread.setDaemon(true);
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

	// --- OLD AzimuthSubsystemV2 servo-PID thread (never started in active code) ---
	// public void startAzimuthPIDThread() { ... }
	// public void stopAzimuthPIDThread() { ... }

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
		sorterThread.setDaemon(true);
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
		launcherPIDThread.setDaemon(true);
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
	 * Starts the background distance sensor read thread (~50 Hz normally,
	 * slows to ~5 Hz when the sensor appears stuck to allow it to self-recover).
	 * <p>
	 * The REV 2 m Distance Sensor (VL53L0X) communicates over I2C and does NOT
	 * participate in the LynxModule hardware bulk-read. Moving the read here
	 * keeps the main loop non-blocking and prevents I2C traffic from
	 * interfering with the bulk-read cache.
	 * <p>
	 * When the raw reading is above {@code MDOConstants.BallSensorMaxValidDistanceCm}
	 * for {@code MDOConstants.BallSensorStuckThreshold} consecutive cycles the
	 * thread backs off to 200 ms between reads, giving the VL53L0X time to
	 * recover from its stuck-at-800+ state without flooding the I2C bus.
	 */
	public void startDistanceSensorThread() {
		if (ballDistanceSensorThreadRunning || ballDistanceSensor == null) {
			return;
		}

		ballDistanceSensorThreadRunning = true;
		ballDistanceSensorThread = new Thread(() -> {
			int consecutiveHighReadings = 0;
			while (ballDistanceSensorThreadRunning) {
				try {
					double raw = ballDistanceSensor.getDistance(
							org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM);

					if (Double.isNaN(raw) || Double.isInfinite(raw) || raw > MDOConstants.BallSensorMaxValidDistanceCm) {
						consecutiveHighReadings++;
						cachedBallDistanceCm = raw; // still publish so callers can see out-of-range

						if (consecutiveHighReadings >= MDOConstants.BallSensorStuckThreshold) {
							ballSensorStuck = true;
							// Cap counter to prevent integer overflow
							consecutiveHighReadings = MDOConstants.BallSensorStuckThreshold;
							// Back off to ~5 Hz — gives the VL53L0X time to self-recover
							// between measurement cycles without flooding the I2C bus
							Thread.sleep(200);
						} else {
							Thread.sleep(20);
						}
					} else {
						// Valid reading — clear stuck state and publish
						consecutiveHighReadings = 0;
						ballSensorStuck = false;
						cachedBallDistanceCm = raw;
						Thread.sleep(20); // ~50 Hz
					}
				} catch (InterruptedException e) {
					Thread.currentThread().interrupt();
					break;
				} catch (Exception e) {
					System.err.println("Error in distance sensor thread: " + e.getMessage());
					try {
						Thread.sleep(50);
					} catch (InterruptedException ie) {
						Thread.currentThread().interrupt();
						break;
					}
				}
			}
		});
		ballDistanceSensorThread.setName("BallDistSensor");
		ballDistanceSensorThread.setDaemon(true);
		ballDistanceSensorThread.start();
	}

	/** Stops the background distance sensor read thread. */
	public void stopDistanceSensorThread() {
		ballDistanceSensorThreadRunning = false;
		if (ballDistanceSensorThread != null) {
			try {
				ballDistanceSensorThread.join(100);
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
			}
		}
	}

	/** Updates the field-centric flag. Call from main loop each iteration. */
	public void setDriveFieldCentric(boolean fieldCentric) {
		this.driveFieldCentric = fieldCentric;
	}

	// startDriveThread / stopDriveThread removed — the follower update thread
	// reads liveGamepad directly, eliminating the extra polling hop.

	/** No-op — drive is handled directly inside the follower update thread. */
	public void startDriveThread() {}

	/** No-op. */
	public void stopDriveThread() {}

	/**
	 * Starts the AprilTag processing thread.
	 * Processes AprilTag detections in background at configurable frequency.
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
	 * Stops the AprilTag processing thread.
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
	 * Starts the follower update thread.
	 * This offloads follower.update() to run continuously in background.
	 * This is the KEY to improving loop performance — follower.update() is expensive.
	 */
	public void startFollowerUpdateThread() {
		if (followerUpdateThreadRunning) {
			return;
		}

		followerUpdateThreadRunning = true;
		followerUpdateThread = new Thread(() -> {
			while (followerUpdateThreadRunning) {
				try {
					synchronized (followerLock) {
						// Fresh encoder data before every follower.update().
						HubInitializer.clearBulkCache();

						// Apply gamepad drive inputs right before update() so motors always
						// see the most recent stick state.
						if (!driveAutomatedActive) {
							Gamepad gp = liveGamepad;
							if (gp != null) {
								follower.setTeleOpDrive(
										-gp.left_stick_y,
										-gp.left_stick_x,
										gp.right_stick_x * -0.75,
										driveFieldCentric);
							}
						}
						// Update odometry, path following, and write motor powers.
						follower.update();
						// Cache isBusy so main loop can read it without acquiring followerLock.
						cachedFollowerBusy = follower.isBusy();
						// Update cached pose immediately after update
						Pose currentPose = follower.getPose();
						if (currentPose != null) {
							cachedPose = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
						}
					}

					// 2 ms sleep — ~500 Hz ceiling.
					Thread.sleep(2);
				} catch (InterruptedException e) {
					Thread.currentThread().interrupt();
					break;
				} catch (Exception e) {
					System.err.println("Error in follower update thread: " + e.getMessage());
				}
			}
		});
		// MAX_PRIORITY so this thread is never starved by the drive/azimuth/aiming threads
		// that also run at MAX_PRIORITY. Without this the follower (which is the only code
		// that actually writes motor powers) gets preempted and effective update rate drops
		// well below 200 Hz, causing perceived stick-to-wheel latency.
		followerUpdateThread.setPriority(Thread.MAX_PRIORITY);
		followerUpdateThread.setName("FollowerUpdate-RT");
		followerUpdateThread.setDaemon(true);
		followerUpdateThread.start();
	}

	/**
	 * Stops the follower update thread.
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

	// ===== SUBSYSTEM UPDATE THREAD =====
	// Moves all non-drive subsystem updates (launcher, elevation, intake, turret) to a background
	// thread so the main loop stays focused on drive input and followers. This eliminates I2C
	// contention in the hot path and improves perceived drive latency.

	private Thread subsystemUpdateThread;
	private volatile boolean subsystemUpdateThreadRunning = false;
	private LauncherSubsystem launcherSubsystem;
	private ElevationSubsystemV2 elevationSubsystem;
	private IntakeSubsystem intakeSubsystem;
	private TurretSubsystem turretSubsystem;

	/**
	 * Sets the launcher subsystem for background update thread.
	 */
	public void setLauncherSubsystem(LauncherSubsystem launcher) {
		this.launcherSubsystem = launcher;
	}

	/**
	 * Sets the elevation subsystem for background update thread.
	 */
	public void setElevationSubsystem(ElevationSubsystemV2 elevation) {
		this.elevationSubsystem = elevation;
	}

	/**
	 * Sets the intake subsystem for background update thread.
	 */
	public void setIntakeSubsystem(IntakeSubsystem intake) {
		this.intakeSubsystem = intake;
	}

	/**
	 * Sets the turret subsystem for background update thread.
	 */
	public void setTurretSubsystem(TurretSubsystem turret) {
		this.turretSubsystem = turret;
	}

	/**
	 * Starts the subsystem update thread. Calls update() on all registered subsystems
	 * at 50Hz to keep their state synchronized without blocking the main drive loop.
	 */
	public void startSubsystemUpdateThread() {
		if (subsystemUpdateThreadRunning) {
			return;
		}

		subsystemUpdateThreadRunning = true;
		subsystemUpdateThread = new Thread(() -> {
			while (subsystemUpdateThreadRunning) {
				try {
					// Update subsystems at 50Hz (20ms) - low enough to not starve follower,
					// fast enough for responsive subsystem control
					if (launcherSubsystem != null) {
						launcherSubsystem.updateRapidFire(sorterController);
						launcherSubsystem.update(0); // RPM will be set by gamepad/turret thread
					}
					if (elevationSubsystem != null) {
						// Elevation updates are deferred to be less frequent
					}
					if (intakeSubsystem != null) {
						intakeSubsystem.update(sorterController.getCachedBallCount(), cachedBallDistanceCm);
					}
					if (sorterController != null) {
						sorterController.lifterUpdater();
					}

					Thread.sleep(20); // ~50Hz
				} catch (InterruptedException e) {
					Thread.currentThread().interrupt();
					break;
				} catch (Exception e) {
					System.err.println("Error in subsystem update thread: " + e.getMessage());
				}
			}
		});
		subsystemUpdateThread.setName("SubsystemUpdate");
		subsystemUpdateThread.setDaemon(true);
		subsystemUpdateThread.start();
	}

	/**
	 * Stops the subsystem update thread.
	 */
	public void stopSubsystemUpdateThread() {
		subsystemUpdateThreadRunning = false;
		if (subsystemUpdateThread != null) {
			try {
				subsystemUpdateThread.join(100);
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
			}
		}
	}
}

