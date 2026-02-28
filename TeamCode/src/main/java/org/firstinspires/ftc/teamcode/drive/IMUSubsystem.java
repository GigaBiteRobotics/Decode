package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * IMUSubsystem - Manages the external IMU for heading/orientation data.
 */
public class IMUSubsystem {

	private final IMU imu;

	/**
	 * Initialize the external IMU with the given orientation parameters.
	 *
	 * @param hardwareMap    The robot's hardware map
	 * @param imuName        The hardware config name for the IMU (e.g., "imuEX")
	 * @param logoDirection  Which direction the REV hub logo faces
	 * @param usbDirection   Which direction the REV hub USB port faces
	 */
	public IMUSubsystem(HardwareMap hardwareMap, String imuName,
						RevHubOrientationOnRobot.LogoFacingDirection logoDirection,
						RevHubOrientationOnRobot.UsbFacingDirection usbDirection) {
		imu = hardwareMap.get(IMU.class, imuName);
		IMU.Parameters parameters = new IMU.Parameters(
				new RevHubOrientationOnRobot(logoDirection, usbDirection)
		);
		imu.initialize(parameters);
	}

	/**
	 * Get the current yaw (heading) in degrees.
	 * @return Yaw angle in degrees
	 */
	public double getHeadingDegrees() {
		YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
		return orientation.getYaw(AngleUnit.DEGREES);
	}

	/**
	 * Get the current yaw (heading) in radians.
	 * @return Yaw angle in radians
	 */
	public double getHeadingRadians() {
		YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
		return orientation.getYaw(AngleUnit.RADIANS);
	}

	/**
	 * Reset the yaw angle to zero.
	 */
	public void resetYaw() {
		imu.resetYaw();
	}

	/**
	 * Get the underlying IMU object for advanced use.
	 * @return The IMU hardware device
	 */
	public IMU getIMU() {
		return imu;
	}
}
