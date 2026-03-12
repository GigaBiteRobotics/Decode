package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretManager {
	LaunchCalculationV2 launchCalculator;
	AzimuthSubsystemV2 azimuthSubsystem;
	LauncherMotorSubsystem launcherMotorSubsystem;
	ElevationSubsystemV2 elevationSubsystem;


	public TurretManager(HardwareMap hardwareMap, LaunchCalculationV2 launchCalculator, AzimuthSubsystemV2 azimuthSubsystem, LauncherMotorSubsystem launcherMotorSubsystem, ElevationSubsystemV2 elevationSubsystem) {
		this.azimuthSubsystem = azimuthSubsystem;
		this.elevationSubsystem = elevationSubsystem;
		this.launcherMotorSubsystem = launcherMotorSubsystem;
	}
}
