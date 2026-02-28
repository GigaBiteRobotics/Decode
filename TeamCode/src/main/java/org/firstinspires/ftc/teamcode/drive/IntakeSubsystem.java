package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * IntakeSubsystem - Manages the intake motor, IN/OUT/STOP state machine,
 * and auto-stop logic when 3 balls are collected.
 */
public class IntakeSubsystem {

	private final CustomMotorController intakeMotor;

	public enum IntakeState {
		IN,
		OUT,
		STOP
	}

	private IntakeState intakeRunningState = IntakeState.STOP;
	private int prevBallCount = 0;
	private final ElapsedTime intakeInputTimer = new ElapsedTime();

	public IntakeSubsystem(CustomMotorController intakeMotor) {
		this.intakeMotor = intakeMotor;
	}

	// ===== Simple actions for GamepadEventHandler callbacks =====

	/** Toggle intake IN mode. If already IN, stops. Otherwise starts IN. */
	public void toggleIn() {
		if (intakeRunningState == IntakeState.IN) {
			intakeRunningState = IntakeState.STOP;
		} else {
			intakeRunningState = IntakeState.IN;
		}
	}

	/** Toggle intake OUT mode. If already OUT, stops. Otherwise starts OUT. */
	public void toggleOut() {
		if (intakeRunningState == IntakeState.OUT) {
			intakeRunningState = IntakeState.STOP;
		} else {
			intakeRunningState = IntakeState.OUT;
		}
	}

	/**
	 * Handle gamepad input for intake toggle.
	 * @param aButton gamepad2.a (toggle IN)
	 * @param bButton gamepad2.b (toggle OUT)
	 */
	public void handleInput(boolean aButton, boolean bButton) {
		if (intakeInputTimer.milliseconds() > 300) {
			if (aButton) {
				intakeInputTimer.reset();
				if (intakeRunningState == IntakeState.IN) {
					intakeRunningState = IntakeState.STOP;
				} else {
					intakeRunningState = IntakeState.IN;
				}
			} else if (bButton) {
				intakeInputTimer.reset();
				if (intakeRunningState == IntakeState.OUT) {
					intakeRunningState = IntakeState.STOP;
				} else {
					intakeRunningState = IntakeState.OUT;
				}
			}
		}
	}

	/**
	 * Update the intake motor. Handles auto-stop and applies motor power.
	 * @param currentBallCount Current number of balls detected by the sorter
	 */
	public void update(int currentBallCount) {
		// Auto-Stop: Turn off intake when 3rd ball is collected
		if (intakeRunningState == IntakeState.IN && currentBallCount >= 3 && prevBallCount < 3) {
			intakeRunningState = IntakeState.STOP;
		}
		prevBallCount = currentBallCount;

		switch (intakeRunningState) {
			case IN:
				intakeMotor.setPower(1);
				break;
			case OUT:
				intakeMotor.setPower(-1);
				break;
			case STOP:
			default:
				intakeMotor.setPower(0);
				break;
		}
	}

	public IntakeState getState() {
		return intakeRunningState;
	}

	public CustomMotorController getMotorController() {
		return intakeMotor;
	}
}
