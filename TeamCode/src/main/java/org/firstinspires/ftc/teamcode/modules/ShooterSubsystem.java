package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.constants.MDOConstants;

/**
 * Shooter subsystem featuring dual flywheel motors and adjustable pitch.
 * <p>
 * Uses a Limelight 3A to determine target distance via the "Target Area" (Ta) metric.
 * Employs a PIDF controller to maintain flywheel velocity and power-law / polynomial
 * regressions to map target area to optimal RPM and pitch angles.
 * </p>
 * <p>
 * Enable via {@link MDOConstants#EnableShooterSubsystem}.
 * PIDF and regression coefficients are live-tunable from FTC Dashboard
 * through the {@code Shooter*} fields on {@link MDOConstants}.
 * </p>
 */
public class ShooterSubsystem {

	private final DcMotorEx topFlywheelMotor;
	private final DcMotorEx bottomFlywheelMotor;
	private final Servo pitchServo;
	private final Limelight3A limelight;
	private final CustomTelemetry telemetryC;

	private double filteredTa = 0.0;

	// TODO: Recalculate PID Constants. These values should be close to final.
	// Pulling from MDOConstants so they can be tuned live on FTC Dashboard.
	private final PIDFController velocityController = new PIDFController(
			MDOConstants.ShooterVelocityP,
			MDOConstants.ShooterVelocityI,
			MDOConstants.ShooterVelocityD,
			MDOConstants.ShooterVelocityF
	);

	/** Ticks per revolution of the 1:1 5203-series bare motor. */
	private static final double TICKS_PER_REV = 28.0;

	/** Maximum safe pitch servo position to avoid mechanical damage. */
	private static final double PITCH_SERVO_MAX = 0.7;

	/**
	 * Constructs a new ShooterSubsystem.
	 *
	 * @param hardwareMap The hardware map to retrieve motor, servo, and camera devices.
	 * @param isRed       {@code true} for Red alliance (Limelight pipeline 1),
	 *                    {@code false} for Blue (pipeline 0).
	 * @param telemetryC  Instance of {@link CustomTelemetry} for real-time data logging.
	 */
	public ShooterSubsystem(HardwareMap hardwareMap, boolean isRed, CustomTelemetry telemetryC) {
		// Initialize flywheel motors
		topFlywheelMotor = hardwareMap.get(DcMotorEx.class, "launcher0");
		topFlywheelMotor.setDirection(DcMotorEx.Direction.REVERSE);
		topFlywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		bottomFlywheelMotor = hardwareMap.get(DcMotorEx.class, "launcher1");
		bottomFlywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		// Initialize pitch servo
		pitchServo = hardwareMap.get(Servo.class, "elevationServo");

		// Initialize Limelight and select the alliance-specific detection pipeline
		limelight = hardwareMap.get(Limelight3A.class, "limelight");
		limelight.pipelineSwitch(isRed ? 1 : 0);
		limelight.start();

		this.telemetryC = telemetryC;
	}

	/**
	 * Executes one cycle of the auto-shooting sequence.
	 * <p>
	 * If a valid Limelight target is visible:
	 * <ol>
	 *   <li>Reads (and optionally low-pass filters) the Target Area (Ta).</li>
	 *   <li>Calculates current RPM from the top flywheel motor encoder velocity.</li>
	 *   <li>Uses a power-law regression to find the target RPM from {@code Ta}.</li>
	 *   <li>Updates the PIDF controller and commands flywheel motor power.</li>
	 *   <li>Uses a quadratic regression to find the target pitch from {@code Ta}.</li>
	 *   <li>Commands the pitch servo (capped at {@value #PITCH_SERVO_MAX}).</li>
	 * </ol>
	 * If no target is visible the flywheels are cut to zero power.
	 * </p>
	 * <p>Call every loop iteration while auto-shooting is active.</p>
	 */
	public void accelerate() {
		limelight.start();
		LLResult result = limelight.getLatestResult();

		if (result != null && result.isValid()) {
			// --- Distance estimation from Target Area ---
			// Optional low-pass filter (uncomment to smooth noisy readings):
			// filteredTa = filteredTa == 0 ? result.getTa() : (0.6 * result.getTa()) + (0.4 * filteredTa);
			filteredTa = result.getTa();

			// --- RPM calculation: power-law regression against Ta ---
			// TODO: recalibrate coefficients — current fit: RPM = A * Ta^B
			double currentRPM = toRPM(topFlywheelMotor.getVelocity());
			double targetRPM = MDOConstants.ShooterRPMCoeffA
					* Math.pow(filteredTa, MDOConstants.ShooterRPMCoeffB);

			// Sync PIDF gains from MDOConstants to allow live dashboard tuning
			velocityController.setPIDF(
					MDOConstants.ShooterVelocityP,
					MDOConstants.ShooterVelocityI,
					MDOConstants.ShooterVelocityD,
					MDOConstants.ShooterVelocityF
			);
			double power = velocityController.calculate(currentRPM, targetRPM);

			// --- Pitch calculation: quadratic regression against Ta ---
			// TODO: recalibrate coefficients — current fit: pitch = C*Ta² + D*Ta + E
			double targetPitch = (MDOConstants.ShooterPitchCoeffC * filteredTa * filteredTa)
					+ (MDOConstants.ShooterPitchCoeffD * filteredTa)
					+ MDOConstants.ShooterPitchCoeffE;

			// --- Telemetry ---
			telemetryC.addData("Shooter Ta", filteredTa);
			telemetryC.addData("Shooter Target RPM", targetRPM);
			telemetryC.addData("Shooter Current RPM", currentRPM);
			telemetryC.addData("Shooter Flywheel Power", power);
			telemetryC.addData("Shooter RPM Error", targetRPM - currentRPM);
			telemetryC.addData("Shooter Target Pitch", targetPitch);

			// --- Apply outputs ---
			topFlywheelMotor.setPower(power);
			bottomFlywheelMotor.setPower(power);
			pitchServo.setPosition(Math.min(targetPitch, PITCH_SERVO_MAX));
		} else {
			topFlywheelMotor.setPower(0);
			bottomFlywheelMotor.setPower(0);
		}
	}

	/**
	 * Returns the most recent filtered Target Area value read from the Limelight.
	 */
	public double getFilteredTa() {
		return filteredTa;
	}

	/**
	 * Shuts down the shooter subsystem.
	 * Pauses the Limelight sensor and cuts power to the flywheel motors.
	 */
	public void idle() {
		topFlywheelMotor.setPower(0);
		bottomFlywheelMotor.setPower(0);
		limelight.pause();
	}

	// ---------------------------------------------------------------------------

	/** Converts encoder ticks/second to RPM for the 1:1 5203-series bare motor. */
	private double toRPM(double ticksPerSecond) {
		return (ticksPerSecond * 60.0) / TICKS_PER_REV;
	}
}

