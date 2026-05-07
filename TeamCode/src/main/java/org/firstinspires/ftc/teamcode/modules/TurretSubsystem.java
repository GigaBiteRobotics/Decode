package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;

/**
 * Turret subsystem powered by dual Continuous Rotation (CR) servos.
 * <p>
 * Utilizes a Limelight 3A camera to automatically track and
 * align with targets using a PIDF control loop based on the target's horizontal offset (Tx).
 * </p>
 */
public class TurretSubsystem {
	private final CRServo leftTurretServo;
	private final CRServo rightTurretServo;
	private final Limelight3A limelight;
	private final CustomTelemetry telemetryC;

	/** PIDF tuned to center Limelight Tx to 0. */
	private final PIDFController limelightPIDF = new PIDFController(0.009, 0.014, 0.0003, 0.0);

	/**
	 * Constructs a new TurretSubsystem.
	 *
	 * @param hardwareMap The hardware map to retrieve servo and camera devices.
	 * @param isRed       {@code true} for the Red alliance (Limelight pipeline 1),
	 *                    {@code false} for Blue (pipeline 0).
	 * @param telemetryC  Instance of {@link CustomTelemetry} for real-time data logging.
	 */
	public TurretSubsystem(HardwareMap hardwareMap, boolean isRed, CustomTelemetry telemetryC) {
		leftTurretServo = hardwareMap.get(CRServo.class, "azimuthServo0");
		rightTurretServo = hardwareMap.get(CRServo.class, "azimuthServo1");

		leftTurretServo.setDirection(CRServo.Direction.REVERSE);
		rightTurretServo.setDirection(CRServo.Direction.REVERSE);

		// Select pipeline based on alliance color
		limelight = hardwareMap.get(Limelight3A.class, "limelight");
		limelight.pipelineSwitch(isRed ? 1 : 0);
		limelight.start();

		this.telemetryC = telemetryC;
	}

	/**
	 * Activates Limelight-based aiming.
	 * <p>
	 * If a valid target is detected, the turret drives the CR servos with power
	 * calculated by the PIDF loop to bring Tx to 0. Power is clamped to ±0.6.
	 * If no target is found, servos are stopped.
	 * </p>
	 * <p>
	 * Call this every loop iteration while auto-aiming is desired.
	 * </p>
	 */
	public void aim() {
		limelight.start();
		LLResult result = limelight.getLatestResult();

		if (result != null && result.isValid()) {
			double power = limelightPIDF.calculate(result.getTx(), 0);
			power = Math.max(-0.6, Math.min(0.6, power));

			telemetryC.addData("Limelight Tx", result.getTx());
			telemetryC.addData("Turret Power", power);

			leftTurretServo.setPower(power);
			rightTurretServo.setPower(power);
		} else {
			leftTurretServo.setPower(0);
			rightTurretServo.setPower(0);
		}
	}

	/**
	 * Puts the turret into an idle state.
	 * Stops servo movement and pauses the Limelight to reduce processing overhead.
	 */
	public void idle() {
		leftTurretServo.setPower(0);
		rightTurretServo.setPower(0);
		limelight.pause();
	}
}