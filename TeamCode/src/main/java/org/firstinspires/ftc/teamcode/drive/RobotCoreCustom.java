package org.firstinspires.ftc.teamcode.drive;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;

public class RobotCoreCustom {
	private final Follower follower;
	IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
			RevHubOrientationOnRobot.LogoFacingDirection.UP,
			RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
	));
	static IMU imuEX;

	public RobotCoreCustom(HardwareMap hardwareMap, Follower follower) {
		imuEX = hardwareMap.get(IMU.class, "imuEX");
		this.follower = follower;
		//imuEX.initialize(parameters);
	}
	public static Double getExternalHeading() {
		YawPitchRollAngles robotOrientation = imuEX.getRobotYawPitchRollAngles();

		// Extract the yaw angle and convert to degrees
		return robotOrientation.getYaw(AngleUnit.DEGREES);
	}
	public static Double[] localizerLauncherCalc(Follower follower, Double[] target) {
		if (follower == null || target == null || target.length < 3) {
			return null;
		}
		Double[] pose = {follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading()};
		// launch position (x, y, z)
		// target position (x, y, z)
		// launch velocity (in/s)
		// gravity (in/s^2)
		return LocalizationAutoAim.calculateLaunchAngle(
				new Double[]{pose[0], pose[1], 10.0}, // launch position (x, y, z)
				new Double[]{target[0], target[1], target[2]}, // target position (x, y, z)
				250.00, // launch velocity (in/s)
				386.09 // gravity (in/s^2)
		);
	}
	public void drawCurrent(Follower follower) {
		try {
			Drawing.drawRobot(follower.getPose());
			Drawing.sendPacket();
		} catch (Exception e) {
			throw new RuntimeException("Drawing failed " + e);
		}
	}

	public void drawCurrentAndHistory(Follower follower) {
		PoseHistory poseHistory = follower.getPoseHistory();
		Drawing.drawPoseHistory(poseHistory);
		drawCurrent(follower);
	}
	// Get RPM over a fixed time window

	public static class CustomMotor {
		private final DcMotor motor;
		private final double TICKS_PER_REV;
		private final boolean hasEncoder;
		private boolean isRPMMode = false;
		private int targetRPM = 0;
		// PID coefficients
		private CustomPIDFController pidfController = new CustomPIDFController(0.1, 0.01, 0.005, 0.0);

		public CustomMotor(HardwareMap hardwareMap, String motorName) {
			this.TICKS_PER_REV = -1; // Default value, can be changed as needed
			this.hasEncoder = false;
			try {
				this.motor = hardwareMap.get(DcMotor.class, motorName);			} catch (Exception e) {
				throw new IllegalArgumentException("Motor with name " + motorName + " not found in hardware map.");
			}
		}
		public CustomMotor(HardwareMap hardwareMap, String motorName, Boolean hasEncoder, double ticksPerRev, CustomPIDFController pidfController) {
			this.TICKS_PER_REV = ticksPerRev;
			this.pidfController = pidfController;
			try {
				this.motor = hardwareMap.get(DcMotor.class, motorName);
			} catch (Exception e) {
				throw new IllegalArgumentException("Motor with name " + motorName + " not found in hardware map.");
			}
			if (hasEncoder) {
				this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				this.hasEncoder = true;
			} else {
				this.hasEncoder = false;
			}
		}
		public double getRPM() {
			if (!hasEncoder) {
				throw new IllegalStateException("Encoder disabled for this motor: " + motor.getDeviceName());
			}
			int startTicks = motor.getCurrentPosition();
			long startTime = System.nanoTime();

			try { Thread.sleep(100); } catch (InterruptedException ignored) {}

			int endTicks = motor.getCurrentPosition();
			long endTime = System.nanoTime();

			double deltaTicks = endTicks - startTicks;
			double deltaTime = (endTime - startTime) / 1e9; // seconds

			double revs = deltaTicks / TICKS_PER_REV;
			double revsPerSec = revs / deltaTime;
			return revsPerSec * 60.0;
		}
		public void setRPM(int rpm) {
			if (!hasEncoder) {
				throw new IllegalStateException("Encoder disabled for this motor: " + motor.getDeviceName());
			}
			this.targetRPM = rpm;
			this.isRPMMode = true;
		}
		public void updateRPMPID() {
			if (isRPMMode) {
				double currentRPM = getRPM();
				double power = pidfController.calculate(targetRPM, currentRPM, 0, 5);
				power = Math.max(-1.0, Math.min(1.0, power)); // Clamp power to [-1, 1]
				motor.setPower(power);
			}

		}

		public void setPower(double power) {
			motor.setPower(power);
			this.isRPMMode = false;
		}
		public DcMotor getMotor() {
			return motor;
		}

		public void setPidfController(CustomPIDFController pidfController) {
			this.pidfController = pidfController;
		}
	}
}