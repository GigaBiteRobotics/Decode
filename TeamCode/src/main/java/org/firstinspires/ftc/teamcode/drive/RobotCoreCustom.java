package org.firstinspires.ftc.teamcode.drive;

import android.annotation.SuppressLint;

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
		imuEX.initialize(parameters);
	}
	public static double getExternalHeading() {
		YawPitchRollAngles robotOrientation = imuEX.getRobotYawPitchRollAngles();

		// Extract the yaw angle and convert to degrees
		return robotOrientation.getYaw(AngleUnit.DEGREES);
	}
	public static Double[] localizer2Angles(Follower follower, Double[] target) {
		if (follower == null || target == null || target.length < 3) {
			return null;
		}
		double[] pose = {follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading()};
		Double targetCannonHeading = Math.atan2(target[1] - pose[1], target[0] - pose[0]);
		Double targetCannonElevation = LocalizationAutoAim.calculateLaunchAngle(
				pose[0], pose[1], 10,
				target[0], target[1], target[2],
				40, 32.2);
		return new Double[]{targetCannonHeading, targetCannonElevation};
	}

}
