package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * HubInitializer - Configures REV Control/Expansion Hub settings.
 * Call once during init to enable bulk caching for better I2C performance.
 */
public class HubInitializer {

	/**
	 * Initialize all connected Lynx hubs with AUTO bulk caching mode.
	 * This reduces redundant I2C reads by caching values per cycle.
	 *
	 * @param hardwareMap The robot's hardware map
	 */
	public static void initBulkCaching(HardwareMap hardwareMap) {
		List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
		for (LynxModule module : allHubs) {
			module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
		}
	}
}

