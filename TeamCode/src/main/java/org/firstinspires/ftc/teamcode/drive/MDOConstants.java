package org.firstinspires.ftc.teamcode.drive;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class MDOConstants {
    public static double launchPower = 0.8;
    public static boolean useAprilTags = true;
    public static Double[] redTargetLocation = new Double[]{-70.0, 70.0, 40.0};
    public static Double[] blueTargetLocation = new Double[]{70.0, 70.0, 40.0};
    public static Double[] launcherCalcConstants = new Double[]{280.00, 386.09};
    public static Double LifterPositionHigh = 0.8;
    public static Double LifterPositionLow = 0.0;
    public static boolean[] LifterReverseMap = new boolean[]{true, false, false};
    public static int LifterWaitToTopTimerMillis = 200;
    public static Double AzimuthIMUOffset = 60.0;
    public static boolean EnableTurretIMUCorrection = true;
    public static Double AzimuthMultiplier = 1.0;
    public static boolean EnableTurret = true;
    public static boolean EnableLauncherCalcAzimuth = true;
    public static Double ElevationOffset = 0.0;
    public static Double ElevationMultiplier = 1.0 / 30.0;
    public static Double AzimuthWrapAroundOffset = 0.0;
    // Azimuth PID Constants: {P, I, D, F}
    // P: Proportional gain - higher = stronger response to error
    // I: Integral gain - corrects steady-state error over time
    // D: Derivative gain - dampens oscillation/overshoot
    // F: Feedforward - not typically used for position control
    // Scale is 180 degrees, so these values are tuned for degree-based error
    // Reduced P to prevent always-max power and allow proportional control
    // Added D term to dampen oscillation and reduce jitter
    // Example: 45° error × 3.0 P = 135, divided by 180 scale = 0.75 power
    public static double[] AzimuthPIDFConstants = new double[]{4.0, 0.2, 0.001, 0.0};
}
