package org.firstinspires.ftc.teamcode.drive;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
				40.0, // launch velocity (in/s)
				386.09 // gravity (in/s^2)
		);
	}

}
