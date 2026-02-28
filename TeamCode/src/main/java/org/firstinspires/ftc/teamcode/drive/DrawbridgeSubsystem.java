package org.firstinspires.ftc.teamcode.drive;

/**
 * DrawbridgeSubsystem - Manages the drawbridge motor.
 * Thin wrapper for now, provides a consistent subsystem interface for future expansion.
 */
public class DrawbridgeSubsystem {

	private final CustomMotor drawbridgeMotor;

	public DrawbridgeSubsystem(CustomMotor drawbridgeMotor) {
		this.drawbridgeMotor = drawbridgeMotor;
	}

	/**
	 * Set the drawbridge motor power.
	 * @param power Motor power (-1 to 1)
	 */
	public void setPower(double power) {
		drawbridgeMotor.setPower(power);
	}

	/**
	 * Stop the drawbridge motor.
	 */
	public void stop() {
		drawbridgeMotor.setPower(0);
	}

	public CustomMotor getMotor() {
		return drawbridgeMotor;
	}
}

