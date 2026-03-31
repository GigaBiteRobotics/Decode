package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Collections;
import java.util.List;

/**
 * HubInitializer - Configures REV Control/Expansion Hub settings.
 *
 * <p>Uses <b>MANUAL</b> bulk-caching mode: the main loop calls
 * {@link #clearBulkCache()} once at the top of each {@code loop()} so all
 * subsequent hardware reads within that iteration return fresh data from a
 * single bulk transaction.  Background threads that read non-bulk sensors
 * (e.g. the VL53L0X distance sensor) are isolated and cannot accidentally
 * reset the cache mid-loop.</p>
 */
public class HubInitializer {

	/** All hubs discovered at init time. Held so {@link #clearBulkCache()} can iterate them. */
	private static List<LynxModule> allHubs = Collections.emptyList();

	/**
	 * Initialize all connected Lynx hubs with MANUAL bulk caching.
	 * Call once during OpMode {@code init()}.
	 *
	 * @param hardwareMap The robot's hardware map
	 */
	public static void initBulkCaching(HardwareMap hardwareMap) {
		allHubs = hardwareMap.getAll(LynxModule.class);
		for (LynxModule module : allHubs) {
			module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
		}
	}

	/**
	 * Clear the bulk-read cache on every hub.
	 * Call <b>once per main loop iteration</b> before any hardware reads so the
	 * first read triggers a fresh bulk transaction and all subsequent reads within
	 * that iteration use the cached result.
	 */
	public static void clearBulkCache() {
		for (LynxModule module : allHubs) {
			module.clearBulkCache();
		}
	}
}
