package org.firstinspires.ftc.teamcode.modules;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.MDOConstants;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

/**
 * TurretSubsystem - Real-time threaded turret controller with sub-millisecond reaction time.
 * <p>
 * Architecture:
 * - Owns a dedicated high-priority aiming thread (~500-1000Hz) that continuously
 * recalculates the turret angle from the latest pose, launch vectors, and offsets.
 * - All shared state is lock-free using volatile fields — no contention with the main loop.
 * - The main loop feeds data via lightweight volatile writes (setPose, setLaunchVectors, etc.)
 * and the aiming thread picks them up on the next iteration (~0.5-1ms later).
 * - The servo PID thread (in CustomThreads) still runs independently for hardware I/O.
 * <p>
 * Timing:
 * - Aiming recalculation: ~500-1000Hz (pure math, no I2C)
 * - Servo PID loop: ~300-500Hz (limited by I2C analog read)
 * - Total latency from heading change to servo command: <2ms
 */
public class TurretSubsystem {

	private final CustomServoController azimuthServo;

	// === Thread-safe inputs (written by main loop, read by aiming thread) ===
	// Using volatile for lock-free single-writer/single-reader pattern.
	private volatile double inputHeadingRad = 0.0;
	private volatile Double inputLaunchAzimuthRad = null; // null = no valid launch vector
	private volatile boolean inputIsRedSide = false;
	private final AtomicLong manualAzimuthOffsetBits = new AtomicLong(Double.doubleToLongBits(0.0));
	/** Target turret angle (degrees) set by the right stick in Forward Aim Mode. */
	private volatile double forwardAimAngleDeg = 0.0;

	// === Thread-safe outputs (written by aiming thread, read by main loop for telemetry) ===
	private volatile double launchAzimuthDeg = 0.0;
	private volatile double robotFieldRelativeAzimuthDeg = 0.0;
	private volatile double finalAzimuthDeg = 0.0;
	private volatile long lastUpdateNanos = 0;
	private volatile double aimingLoopHz = 0.0;

	// === Thread control ===
	private Thread aimingThread;
	private final AtomicBoolean aimingThreadRunning = new AtomicBoolean(false);

	// === Input timer (for debounced manual adjustment) ===
	private final ElapsedTime azimuthAdjustTimer = new ElapsedTime();

	public TurretSubsystem(CustomServoController azimuthServo) {
		this.azimuthServo = azimuthServo;
	}

	// =========================================================================
	// Thread lifecycle
	// =========================================================================

	/**
	 * Start the real-time aiming thread.
	 * Call once during OpMode start() after all hardware is initialized.
	 */
	public void startThread() {
		if (aimingThreadRunning.get()) return;
		aimingThreadRunning.set(true);

		aimingThread = new Thread(() -> {
			long prevNanos = System.nanoTime();

			while (aimingThreadRunning.get()) {
				try {
					// === Snapshot all volatile inputs (single reads, no tearing) ===
					double headingRad = inputHeadingRad;
					Double launchAzRad = inputLaunchAzimuthRad;
					boolean redSide = inputIsRedSide;
					double manualOffset = getManualAzimuthOffset();

					// === Forward Aim Mode: turret angle driven by right stick ===
					if (MDOConstants.EnableForwardAimMode) {
						double aimAngle = forwardAimAngleDeg;
						finalAzimuthDeg = aimAngle;
						if (MDOConstants.EnableTurret) {
							azimuthServo.setPosition(-aimAngle / 180.0);
						}
						// Still track Hz and timestamp
						long nowNanos2 = System.nanoTime();
						long deltaNanos2 = nowNanos2 - prevNanos;
						if (deltaNanos2 > 0) aimingLoopHz = 1_000_000_000.0 / deltaNanos2;
						prevNanos = nowNanos2;
						lastUpdateNanos = nowNanos2;
						Thread.sleep(0, 500_000);
						continue;
					}

					// === Compute aiming ===
					double headingDeg = Math.toDegrees(headingRad);
					robotFieldRelativeAzimuthDeg = headingDeg;

					double azFineAdjust = redSide
							? MDOConstants.RedAzimuthFineAdjustment
							: MDOConstants.BlueAzimuthFineAdjustment;

					boolean hasLaunchVectors = (launchAzRad != null);
					if (hasLaunchVectors) {
						launchAzimuthDeg = Math.toDegrees(launchAzRad);
					}

					double computedAzimuth = 0.0;

					if (MDOConstants.EnableTurretIMUCorrection && MDOConstants.EnableTurret) {
						if (MDOConstants.EnableLauncherCalcAzimuth && hasLaunchVectors) {
							computedAzimuth = headingDeg - launchAzimuthDeg
									+ MDOConstants.AzimuthIMUOffset
									+ MDOConstants.AzimuthFineAdjustment
									+ azFineAdjust
									+ manualOffset;
						} else {
							computedAzimuth = (headingDeg + MDOConstants.AzimuthIMUOffset + manualOffset)
									* MDOConstants.AzimuthMultiplier;
						}
					}

					// Normalize to [-180, 180]
					computedAzimuth = ((computedAzimuth + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;

					finalAzimuthDeg = computedAzimuth;

					// === Apply to servo ===
					double servoPosition = -computedAzimuth / 180.0;


					if (MDOConstants.EnableTurret) {
						azimuthServo.setPosition(servoPosition);
					} else {
						azimuthServo.setPosition(0.0);
					}

					// === Performance tracking ===
					long nowNanos = System.nanoTime();
					long deltaNanos = nowNanos - prevNanos;
					if (deltaNanos > 0) {
						aimingLoopHz = 1_000_000_000.0 / deltaNanos;
					}
					prevNanos = nowNanos;
					lastUpdateNanos = nowNanos;

					// Yield briefly — pure math loop, no I2C here.
					// setPosition() is a volatile write (instant), servoPidLoop() runs on its own thread.
					// Sleep <1ms to stay near ~1000Hz without burning CPU.
					// Thread.sleep(0, nanosTimeout) on Android is at best ~0.1ms granularity.
					Thread.sleep(0, 500_000); // 0.5ms target

				} catch (InterruptedException e) {
					Thread.currentThread().interrupt();
					break;
				} catch (Exception e) {
					// Don't crash the thread on transient errors
					System.err.println("TurretSubsystem aiming thread error: " + e.getMessage());
				}
			}
		});
		aimingThread.setName("TurretAiming-RT");
		aimingThread.setPriority(Thread.MAX_PRIORITY - 1); // Just below the PID thread
		aimingThread.setDaemon(true);
		aimingThread.start();
	}

	/**
	 * Stop the aiming thread. Call during OpMode stop().
	 */
	public void stopThread() {
		aimingThreadRunning.set(false);
		if (aimingThread != null) {
			try {
				aimingThread.join(100);
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
			}
		}
	}

	// =========================================================================
	// Data feed — call from main loop (lightweight volatile writes)
	// =========================================================================

	/**
	 * Set the desired turret angle (degrees) for Forward Aim Mode.
	 * Driven by the right stick X of gamepad2: stickX * ForwardAimAngleRange.
	 * This is a single volatile write — safe to call every main loop iteration.
	 */
	public void setForwardAimAngleDeg(double deg) {
		forwardAimAngleDeg = deg;
	}

	/**
	 * Feed the latest robot pose to the aiming thread.
	 * Call every main loop iteration. This is a single volatile write (~ns).
	 */
	public void setPose(Pose currentPose) {
		inputHeadingRad = currentPose.getHeading();
	}

	/**
	 * Feed the latest launch vectors to the aiming thread.
	 *
	 * @param launchVectors Result from LauncherCalculations (may be null if target unreachable)
	 */
	public void setLaunchVectors(Double[] launchVectors) {
		inputLaunchAzimuthRad = (launchVectors != null) ? launchVectors[1] : null;
	}

	/**
	 * Set the alliance side. Only needs to be called when it changes.
	 */
	public void setRedSide(boolean isRedSide) {
		inputIsRedSide = isRedSide;
	}

	/**
	 * Combined lightweight update — feeds pose and launch vectors in one call.
	 * Replaces the old synchronous update() method.
	 * All work is done by volatile writes; the aiming thread picks them up in <1ms.
	 *
	 * @param currentPose   The current robot pose (thread-safe copy)
	 * @param launchVectors The launch vectors from LauncherCalculations (may be null)
	 * @param isRedSide     Whether the robot is on the red alliance
	 */
	public void update(Pose currentPose, Double[] launchVectors, boolean isRedSide) {
		inputHeadingRad = currentPose.getHeading();
		inputLaunchAzimuthRad = (launchVectors != null) ? launchVectors[1] : null;
		inputIsRedSide = isRedSide;
	}

	// =========================================================================
	// Manual azimuth adjustment — safe to call from any thread
	// =========================================================================

	/**
	 * Adjust the manual azimuth offset by the given amount (in degrees).
	 *
	 * @param deltaDeg Amount to add to the offset (positive = right, negative = left)
	 */
	public void adjustAzimuth(double deltaDeg) {
		long oldBits, newBits;
		do {
			oldBits = manualAzimuthOffsetBits.get();
			double oldVal = Double.longBitsToDouble(oldBits);
			newBits = Double.doubleToLongBits(oldVal + deltaDeg);
		} while (!manualAzimuthOffsetBits.compareAndSet(oldBits, newBits));
	}

	/**
	 * Reset the manual azimuth offset to zero.
	 */
	public void resetAzimuthOffset() {
		manualAzimuthOffsetBits.set(Double.doubleToLongBits(0.0));
	}

	/**
	 * Handle D-pad input for manual azimuth adjustment.
	 * Call once per loop with the current gamepad state.
	 */
	public void handleInput(boolean dpadLeft, boolean dpadRight, boolean yButton) {
		if (azimuthAdjustTimer.milliseconds() > 150) {
			if (dpadRight) {
				adjustAzimuth(1.0);
				azimuthAdjustTimer.reset();
			} else if (dpadLeft) {
				adjustAzimuth(-1.0);
				azimuthAdjustTimer.reset();
			}
			if (yButton) {
				resetAzimuthOffset();
				azimuthAdjustTimer.reset();
			}
		}
	}

	// =========================================================================
	// Telemetry getters — all volatile reads, safe from any thread
	// =========================================================================

	public double getManualAzimuthOffset() {
		return Double.longBitsToDouble(manualAzimuthOffsetBits.get());
	}

	public double getLaunchAzimuthDeg() {
		return launchAzimuthDeg;
	}

	public double getRobotFieldRelativeAzimuthDeg() {
		return robotFieldRelativeAzimuthDeg;
	}

	public double getFinalAzimuthDeg() {
		return finalAzimuthDeg;
	}

	public double getServoPosition() {
		return azimuthServo.getPosition();
	}

	public double getServoTarget() {
		return azimuthServo.getTargetPosition();
	}

	public CustomServoController getServoController() {
		return azimuthServo;
	}

	/**
	 * Get the real-time aiming loop frequency in Hz.
	 */
	public double getAimingLoopHz() {
		return aimingLoopHz;
	}

	/**
	 * Returns true if the aiming thread is alive and running.
	 */
	public boolean isThreadRunning() {
		return aimingThreadRunning.get() && aimingThread != null && aimingThread.isAlive();
	}
}

