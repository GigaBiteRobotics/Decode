package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.constants.MDOConstants;

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
	private final boolean isRed;

	/** PIDF tuned to center Limelight Tx to 0. */
	private final PIDFController limelightPIDF = new PIDFController(0.009, 0.014, 0.0003, 0.0);

	/** Set by aim() each loop — avoids a second getLatestResult() call in hasLock(). */
	private boolean locked = false;

	/** Target Area from the last valid limelight result. Used by elevation + RPM calc. */
	private double lastTa = 0.0;

	/** Tracks the active pipeline index to avoid redundant pipelineSwitch() calls. */
	private int currentPipeline = -1;

	/**
	 * Constructs a new TurretSubsystem.
	 *
	 * @param hardwareMap The hardware map to retrieve servo and camera devices.
	 * @param isRed       {@code true} for the Red alliance (Limelight pipeline 1),
	 *                    {@code false} for Blue (pipeline 0).
	 * @param telemetryC  Instance of {@link CustomTelemetry} for real-time data logging.
	 */
	public TurretSubsystem(HardwareMap hardwareMap, boolean isRed, CustomTelemetry telemetryC) {
		this.isRed = isRed;
		leftTurretServo = hardwareMap.get(CRServo.class, "azimuthServo0");
		rightTurretServo = hardwareMap.get(CRServo.class, "azimuthServo1");

		leftTurretServo.setDirection(CRServo.Direction.REVERSE);
		rightTurretServo.setDirection(CRServo.Direction.REVERSE);

		limelight = hardwareMap.get(Limelight3A.class, "limelight");
		// Start on the idle pipeline (low-FPS, low-res) to keep the limelight warm
		// without running the heavy alliance detection pipeline.
		switchPipeline(MDOConstants.LimelightPipelineIdle);
		limelight.start();

		this.telemetryC = telemetryC;
	}

	/**
	 * Activates Limelight-based aiming. Call every loop while the aim button is held.
	 * Also updates the internal lock state read by {@link #hasLock()}.
	 */
	public void aim() {
		// Switch to the alliance detection pipeline only when needed.
		int alliancePipeline = isRed ? MDOConstants.LimelightPipelineRed : MDOConstants.LimelightPipelineBlue;
		switchPipeline(alliancePipeline);
		limelight.start();
		LLResult result = limelight.getLatestResult();

		if (result != null && result.isValid()) {
			lastTa = result.getTa();
			double power = limelightPIDF.calculate(result.getTx(), 0);
			power = Math.max(-0.6, Math.min(0.6, power));

			telemetryC.addData("Turret", "LOCKED");
			telemetryC.addData("Limelight Tx", result.getTx());
			telemetryC.addData("Turret Power", power);

			rightTurretServo.setPower(power);
			leftTurretServo.setPower(power);
			locked = true;
		} else {
			telemetryC.addData("Turret", result == null ? "NULL result" : "No target (isValid=false)");
			rightTurretServo.setPower(0);
			leftTurretServo.setPower(0);
			locked = false;
		}
	}

	/**
	 * Returns {@code true} if the last {@link #aim()} call had a valid target.
	 */
	public boolean hasLock() {
		return locked;
	}

	/**
	 * Returns the Target Area (Ta) from the last valid limelight result.
	 * Used to calculate distance-based RPM and elevation pitch.
	 */
	public double getTa() {
		return lastTa;
	}

	/**
	 * Puts the turret into an idle state.
	 * Stops servo movement and switches the Limelight back to the low-FPS idle pipeline.
	 */
	public void idle() {
		rightTurretServo.setPower(0);
		leftTurretServo.setPower(0);
		switchPipeline(MDOConstants.LimelightPipelineIdle);
		locked = false;
	}

	/** Only calls pipelineSwitch() when the pipeline actually needs to change. */
	private void switchPipeline(int index) {
		if (currentPipeline != index) {
			limelight.pipelineSwitch(index);
			currentPipeline = index;
		}
	}
}