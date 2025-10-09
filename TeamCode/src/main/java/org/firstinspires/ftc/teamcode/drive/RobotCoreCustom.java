package org.firstinspires.ftc.teamcode.drive;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotCoreCustom {
	IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
			RevHubOrientationOnRobot.LogoFacingDirection.UP,
			RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
	));
	static IMU imuEX;

	public RobotCoreCustom(HardwareMap hardwareMap) {
		imuEX = hardwareMap.get(IMU.class, "imuEX");
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
	// Get RPM over a fixed time window

	public static class CustomMotor {
		private final DcMotor motor;
		private final double TICKS_PER_REV;
		private final boolean encoderEnabled;

		public CustomMotor(HardwareMap hardwareMap, String motorName) {
			this.TICKS_PER_REV = -1; // Default value, can be changed as needed
			this.encoderEnabled = false;
			try {
				this.motor = hardwareMap.get(DcMotor.class, motorName);			} catch (Exception e) {
				throw new IllegalArgumentException("Motor with name " + motorName + " not found in hardware map.");
			}
		}
		public CustomMotor(HardwareMap hardwareMap, String motorName, Boolean hasEncoder, double ticksPerRev) {
			this.TICKS_PER_REV = ticksPerRev;
			try {
				this.motor = hardwareMap.get(DcMotor.class, motorName);
			} catch (Exception e) {
				throw new IllegalArgumentException("Motor with name " + motorName + " not found in hardware map.");
			}
			if (hasEncoder) {
				this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				this.encoderEnabled = true;
			} else {
				this.encoderEnabled = false;
			}
		}
		public double getRPM() {
			if (!encoderEnabled) {
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
		public void setPower(double power) {
			motor.setPower(power);
		}
		public DcMotor getMotor() {
			return motor;
		}
	}
}
