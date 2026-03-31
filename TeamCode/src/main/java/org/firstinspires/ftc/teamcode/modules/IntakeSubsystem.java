package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.constants.MDOConstants;

/**
 * IntakeSubsystem — Controls the ball intake motor with a simple IN/OUT/STOP state machine.
 *
 * <p>The subsystem is designed to be updated once per loop by calling
 * {@link #update(int, double)} with the current ball count (from {@link CustomSorterController})
 * and the latest distance reading from the background sensor thread
 * ({@link CustomThreads#getCachedBallDistanceCm()}).  Neither value is read directly
 * from hardware here, keeping the main loop free of blocking I2C calls.</p>
 *
 * <h3>Auto-stop behaviour:</h3>
 * <ul>
 *   <li>When running IN, the intake stops automatically once 3 balls are loaded
 *       ({@code ballCount >= 3}).</li>
 * </ul>
 *
 * <h3>Usage example:</h3>
 * <pre>{@code
 * // In init():
 * intake = new IntakeSubsystem(intakeMotor, ballDistSensor);
 *
 * // Bind gamepad buttons (GamepadEventHandler):
 * gp2Handler.onPress("intakeIn",  gp -> gp.a, () -> intake.toggleIn());
 * gp2Handler.onPress("intakeOut", gp -> gp.b, () -> intake.toggleOut());
 *
 * // In loop():
 * intake.update(sorterController.getCachedBallCount(),
 *               customThreads.getCachedBallDistanceCm());
 * }</pre>
 */
public class IntakeSubsystem {

	// ===== State =====

	public enum IntakeState {
		IN,
		OUT,
		STOP
	}

	// ===== Fields =====

	private final CustomMotorController intakeMotor;

	/** Kept for potential direct-read fallback; not called during normal operation. */
	@SuppressWarnings("FieldCanBeLocal")
	private final DistanceSensor distanceSensor;

	private IntakeState state = IntakeState.STOP;

	/** Most recent distance value passed to {@link #update}; returned by {@link #GetDistanceBallSensorCM()}. */
	private double lastDistanceCm = 9999.0;

	/** Timer used to run the intake automatically for {@link MDOConstants#BallIntakeTimerMs} ms. */
	private final ElapsedTime autoRunTimer = new ElapsedTime();
	/** True while the intake is being driven by the distance-sensor auto-run logic. */
	private boolean autoRunActive = false;
	/**
	 * Re-arms the distance trigger once the ball clears the detection zone.
	 * Prevents the auto-run from re-firing continuously while an object stays
	 * inside the threshold after the timed run has already finished.
	 */
	private boolean distanceTriggerArmed = true;

	// ===== Constructor =====

	/**
	 * @param intakeMotor    Motor controller for the intake wheel(s).
	 * @param distanceSensor The "frontRange" REV 2 m sensor used to detect approaching balls.
	 *                       Its readings must be supplied via
	 *                       {@link CustomThreads#getCachedBallDistanceCm()} to avoid blocking the
	 *                       main loop — do not call getDistance() from here during teleop.
	 */
	public IntakeSubsystem(CustomMotorController intakeMotor, DistanceSensor distanceSensor) {
		this.intakeMotor = intakeMotor;
		this.distanceSensor = distanceSensor;
	}

	// ===== Toggle helpers (for GamepadEventHandler callbacks) =====

	/**
	 * Toggle the intake between IN and STOP.
	 * If currently OUT, also switches to IN.
	 * Always cancels any active distance-sensor auto-run so manual input is
	 * immediately honoured.
	 */
	public void toggleIn() {
		autoRunActive = false; // driver override always wins
		if (state == IntakeState.IN) {
			state = IntakeState.STOP;
		} else {
			state = IntakeState.IN;
		}
	}

	/**
	 * Toggle the intake between OUT and STOP.
	 * If currently IN, also switches to OUT.
	 * Always cancels any active distance-sensor auto-run.
	 */
	public void toggleOut() {
		autoRunActive = false; // driver override always wins
		if (state == IntakeState.OUT) {
			state = IntakeState.STOP;
		} else {
			state = IntakeState.OUT;
		}
	}

	/** Immediately stops the intake regardless of current state. */
	public void stop() {
		autoRunActive = false;
		state = IntakeState.STOP;
	}

	// ===== Main update =====

	/**
	 * Update intake state and motor output.  Call once per main loop iteration.
	 *
	 * @param ballCount  Current number of balls in the sorter (from
	 *                   {@link CustomSorterController#getCachedBallCount()}).
	 * @param distanceCm Latest distance reading in centimetres from the background
	 *                   sensor thread ({@link CustomThreads#getCachedBallDistanceCm()}).
	 */
	public void update(int ballCount, double distanceCm) {
		lastDistanceCm = distanceCm;

		// Auto-stop when all sorter slots are full
		if (state == IntakeState.IN && ballCount >= 3) {
			state = IntakeState.STOP;
		}

		// Re-arm once the ball has cleared the detection zone
		if (!distanceTriggerArmed && distanceCm >= MDOConstants.BallDetectionDistanceCm) {
			distanceTriggerArmed = true;
		}

		// Auto-run: rising edge only — ball just entered range, intake idle, room available
		if (distanceTriggerArmed
				&& !autoRunActive
				&& state == IntakeState.STOP
				&& ballCount < 3
				&& distanceCm < MDOConstants.BallDetectionDistanceCm) {
			autoRunActive = true;
			distanceTriggerArmed = false; // disarm until ball leaves range again
			autoRunTimer.reset();
		}

		// Cancel auto-run once the timer expires, sorter is full, or driver takes over
		if (autoRunActive && (autoRunTimer.milliseconds() >= MDOConstants.BallIntakeTimerMs
				|| ballCount >= 3
				|| state == IntakeState.OUT)) {
			autoRunActive = false;
		}

		// Apply motor power — auto-run overrides STOP, but manual IN/OUT always wins
		if (autoRunActive) {
			intakeMotor.setPower(MDOConstants.IntakePower);
		} else {
			switch (state) {
				case IN:
					intakeMotor.setPower(MDOConstants.IntakePower);
					break;
				case OUT:
					intakeMotor.setPower(-MDOConstants.IntakePower);
					break;
				case STOP:
				default:
					intakeMotor.setPower(0.0);
					break;
			}
		}
	}

	// ===== Getters for telemetry =====

	/** Returns the current intake state. */
	public IntakeState getState() {
		return state;
	}

	/**
	 * Returns the most recent distance reading (cm) supplied to {@link #update}.
	 * Non-blocking — wraps the value that came from the background sensor thread.
	 */
	public double GetDistanceBallSensorCM() {
		return lastDistanceCm;
	}

	/** Returns {@code true} while the intake is being automatically driven by the distance sensor. */
	public boolean isAutoRunActive() {
		return autoRunActive;
	}
}

