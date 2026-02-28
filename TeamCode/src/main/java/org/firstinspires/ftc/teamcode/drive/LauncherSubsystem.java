package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * LauncherSubsystem - Manages launcher motors, spinning state, PID/manual mode,
 * reverse mode, pooping mode, and RPM calculations.
 */
public class LauncherSubsystem {

	private final CustomMotorController launcherMotors;

	// State
	private boolean launcherSpinning = false;
	private boolean launcherPooping = false;
	private boolean launcherReverseActive = false;
	private boolean prevLeftTriggerPressed = false;
	private double targetPower = 0.0;

	// Rapid fire state
	private boolean rapidFireActive = false;
	private int rapidFireIndex = 0;
	private final ElapsedTime rapidFireTimer = new ElapsedTime();
	private boolean rapidFireTriggerWasPressed = false;

	public LauncherSubsystem(CustomMotorController launcherMotors) {
		this.launcherMotors = launcherMotors;
	}

	// ===== Simple actions for GamepadEventHandler callbacks =====

	/** Toggle the launcher spinning on/off. */
	public void toggleSpin() {
		if (!launcherSpinning) {
			targetPower = MDOConstants.launchPower;
			launcherSpinning = true;
		} else {
			targetPower = 0;
			launcherSpinning = false;
		}
	}

	/** Toggle the launcher poop mode on/off. */
	public void togglePoop() {
		launcherPooping = !launcherPooping;
	}

	/** Toggle the launcher reverse mode on/off. */
	public void toggleReverse() {
		launcherReverseActive = !launcherReverseActive;
	}

	/**
	 * Start a rapid fire sequence (fires all pits in order with delays).
	 * Call once to begin the sequence, then call {@link #updateRapidFire(CustomSorterController)}
	 * every loop to advance it.
	 */
	public void startRapidFire(CustomSorterController sorterController) {
		if (!rapidFireActive) {
			rapidFireActive = true;
			rapidFireIndex = 0;
			rapidFireTimer.reset();
			// Fire first ball immediately
			int pitToFire = MDOConstants.RapidFireOrder[rapidFireIndex];
			sorterController.launchFromPit(pitToFire);
			rapidFireIndex++;
		}
	}

	/**
	 * Continue the rapid fire sequence if active.
	 * Must be called every loop for the sequence to advance.
	 */
	public void updateRapidFire(CustomSorterController sorterController) {
		if (rapidFireActive && rapidFireIndex < MDOConstants.RapidFireOrder.length) {
			if (rapidFireTimer.milliseconds() >= MDOConstants.RapidFireDelayMs) {
				int pitToFire = MDOConstants.RapidFireOrder[rapidFireIndex];
				sorterController.launchFromPit(pitToFire);
				rapidFireIndex++;
				rapidFireTimer.reset();
			}
		}
		// End rapid fire when all pits have been fired
		if (rapidFireIndex >= MDOConstants.RapidFireOrder.length) {
			rapidFireActive = false;
		}
	}

	/**
	 * Handle gamepad input for launcher controls.
	 *
	 * @param dpadDown        gamepad2.dpad_down (toggle spin)
	 * @param dpadUp          gamepad2.dpad_up (toggle poop mode)
	 * @param leftTrigger     gamepad2.left_trigger > 0.5 (toggle reverse)
	 * @param gamepadTimerMs  elapsed time from the main gamepad timer
	 * @param gamepadTimer    the main gamepad timer (will be reset on input)
	 */
	public void handleInput(boolean dpadDown, boolean dpadUp, boolean leftTrigger, double gamepadTimerMs, ElapsedTime gamepadTimer) {
		if (dpadDown && gamepadTimerMs > 300) {
			gamepadTimer.reset();
			if (!launcherSpinning) {
				targetPower = MDOConstants.launchPower;
				launcherSpinning = true;
			} else {
				targetPower = 0;
				launcherSpinning = false;
			}
		}
		if (dpadUp && gamepadTimerMs > 300) {
			gamepadTimer.reset();
			launcherPooping = !launcherPooping;
		}

		// Left trigger toggle: launcher reverse
		if (leftTrigger && !prevLeftTriggerPressed) {
			launcherReverseActive = !launcherReverseActive;
		}
		prevLeftTriggerPressed = leftTrigger;
	}

	/**
	 * Handle rapid fire logic.
	 *
	 * @param triggerPressed    gamepad2.right_trigger > 0.5
	 * @param sorterController  the sorter controller to fire balls from
	 */
	public void handleRapidFire(boolean triggerPressed, CustomSorterController sorterController) {
		// Edge detection: Start rapid fire sequence when trigger is first pressed
		if (triggerPressed && !rapidFireTriggerWasPressed && !rapidFireActive) {
			rapidFireActive = true;
			rapidFireIndex = 0;
			rapidFireTimer.reset();
			// Fire first ball immediately
			int pitToFire = MDOConstants.RapidFireOrder[rapidFireIndex];
			sorterController.launchFromPit(pitToFire);
			rapidFireIndex++;
		}
		rapidFireTriggerWasPressed = triggerPressed;

		// Continue rapid fire sequence if active
		if (rapidFireActive && rapidFireIndex < MDOConstants.RapidFireOrder.length) {
			if (rapidFireTimer.milliseconds() >= MDOConstants.RapidFireDelayMs) {
				int pitToFire = MDOConstants.RapidFireOrder[rapidFireIndex];
				sorterController.launchFromPit(pitToFire);
				rapidFireIndex++;
				rapidFireTimer.reset();
			}
		}

		// End rapid fire when all pits have been fired
		if (rapidFireIndex >= MDOConstants.RapidFireOrder.length) {
			rapidFireActive = false;
		}
	}

	/**
	 * Update launcher motor output based on current state and dynamic RPM.
	 *
	 * @param dynamicRPM The dynamically calculated RPM from LauncherCalculations
	 */
	public void update(int dynamicRPM) {
		// Clamp target power
		targetPower = Math.max(-1.0, Math.min(1.0, targetPower));

		if (MDOConstants.EnableLauncherPID) {
			// PID mode: use setRPM for closed-loop RPM control
			if (launcherReverseActive) {
				launcherMotors.setRPM(-MDOConstants.LauncherReverseRPM);
			} else if (launcherPooping) {
				launcherMotors.setRPM(1200);
			} else {
				launcherMotors.setRPM(launcherSpinning ? dynamicRPM : 0);
			}
			launcherMotors.setPIDFController(MDOConstants.LauncherPIDF);
		} else {
			// Manual power mode: bypass PID and use direct power
			if (launcherReverseActive) {
				launcherMotors.setPower(-MDOConstants.LauncherManualPower);
			} else if (launcherPooping) {
				launcherMotors.setPower(0.2);
			} else {
				launcherMotors.setPower(launcherSpinning ? MDOConstants.LauncherManualPower : 0);
			}
		}
	}

	// ===== Getters for telemetry =====

	public boolean isSpinning() {
		return launcherSpinning;
	}

	public boolean isReverse() {
		return launcherReverseActive;
	}

	public boolean isPooping() {
		return launcherPooping;
	}

	public double getRPM() {
		return launcherMotors.getAverageRPM();
	}

	public String getDebugString() {
		return launcherMotors.getDebugString();
	}

	public double getPIDOutput() {
		return launcherMotors.getPIDOutput();
	}

	public boolean isRapidFireActive() {
		return rapidFireActive;
	}

	public CustomMotorController getMotorController() {
		return launcherMotors;
	}
}

