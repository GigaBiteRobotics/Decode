package org.firstinspires.ftc.teamcode.drive;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class MDOConstants {
    public static boolean usePIDFLauncher = false;
    public static boolean useAprilTags = true;
    public static Double[] targetLocation = new Double[]{-70.0, 70.0, 40.0};
    public static CustomPIDFController launcherPIDF = new CustomPIDFController(1.5, 0.0, 0.2, 0.0);
    public static Double[] launcherCalcConstants = new Double[]{280.00, 386.09};
}
