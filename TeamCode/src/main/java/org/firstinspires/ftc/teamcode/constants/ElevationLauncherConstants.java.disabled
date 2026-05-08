package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

/**
 * ElevationLauncherConstants — All tuning parameters for the physics-based elevation
 * calculation in {@link org.firstinspires.ftc.teamcode.modules.ElevationSubsystemV2}.
 *
 * <p>Every field is {@code public static} so FTC Dashboard can change them at runtime.
 * Never make these fields {@code final}.
 *
 * <h2>Tuning workflow</h2>
 * <ol>
 *   <li>Set {@link #LaunchHeightInches} and {@link #TargetHeightInches} to the real geometry.</li>
 *   <li>Shoot at a known distance, read the servo position on the dashboard, and adjust
 *       {@link #AngleToServoScale} / {@link #AngleToServoOffset} until the servo position
 *       matches the physical sweet-spot.</li>
 *   <li>Sweep across multiple distances and use {@link #DistanceTrimSlope} +
 *       {@link #DistanceTrimIntercept} to correct any systematic trend.</li>
 *   <li>Use {@link #AngleCorrA} / {@link #AngleCorrB} / {@link #AngleCorrC} for a quadratic
 *       correction over the full angle range if a linear map still has curvature error.</li>
 *   <li>Dial in {@link #GlobalElevationTrim} for the last few millimetres of residual error,
 *       then alliance-split with {@link #RedAllianceTrim} / {@link #BlueAllianceTrim}.</li>
 * </ol>
 */
@Config
public class ElevationLauncherConstants {

    // =========================================================================
    // Velocity Model
    // =========================================================================

    /**
     * Launcher RPM at which {@link #BaseVelocity} was characterised.
     * This is the reference operating point for all velocity scaling.
     */
    public static double BaseRPM = 3800.0;

    /**
     * Ball launch velocity (inches/s) measured at {@link #BaseRPM}.
     * Tune by timing a shot at a known distance and height, then back-calculate
     * the velocity from the projectile equations.
     */
    public static double BaseVelocity = 280.0;

    /**
     * Exponent for the RPM → velocity power-law:
     * <pre>  velocity = BaseVelocity * (rpm / BaseRPM) ^ VelocityExponent</pre>
     * 1.0 = linear (default). Values &lt; 1 = sublinear (motor saturates faster);
     * values &gt; 1 = superlinear (useful if the flywheel is in a high-inertia regime).
     */
    public static double VelocityExponent = 1.0;

    /**
     * Flat additive offset (inches/s) applied after the power-law scaling:
     * <pre>  velocity = BaseVelocity * (rpm / BaseRPM)^VelocityExponent + VelocityOffset</pre>
     * Compensates for a systematic bias (e.g. measured velocity > 0 at 0 RPM due to
     * model error or slip).  Negative values lower the effective velocity.
     */
    public static double VelocityOffset = 0.0;

    // =========================================================================
    // Physical Geometry
    // =========================================================================

    /**
     * Effective gravity constant (inches/s²).
     * Standard 1 g ≈ 386.09 in/s².  Reduce to compensate for aerodynamic drag
     * (drag acts like a reduced gravity on flat trajectories) — typical range 300–390.
     */
    public static double Gravity = 386.09;

    /**
     * Height of the launcher exit point above the field floor (inches).
     * Measure from the floor to where the ball leaves the flywheel.
     */
    public static double LaunchHeightInches = 0.0;

    /**
     * Height of the scoring target above the field floor (inches).
     * For a target at a fixed height (e.g. a goal opening), set this once and leave it.
     */
    public static double TargetHeightInches = 0.0;

    // =========================================================================
    // Trajectory Selection
    // =========================================================================

    /**
     * When {@code false} (default): use the <em>low-angle</em> (flat/fast) trajectory.
     * When {@code true}: use the <em>high-angle</em> (lobbed/arcing) trajectory.
     * High-angle is more tolerant of small velocity errors at long range but produces a
     * steeper descent angle and requires higher elevation servo positions.
     */
    public static boolean UseHighAngle = false;

    // =========================================================================
    // Angle → Servo Mapping
    // =========================================================================

    /**
     * Linear scale factor mapping the physical launch angle (radians) to servo position:
     * <pre>  servo_raw = AngleToServoScale * corrected_angle_rad + AngleToServoOffset</pre>
     * Derived from the V1 calibration; typical range 1.2 – 2.0.
     */
    public static double AngleToServoScale = 1.64;

    /**
     * Additive servo-position offset applied after scaling (see {@link #AngleToServoScale}).
     * Derived from the V1 calibration; typical range −0.8 – 0.0.
     */
    public static double AngleToServoOffset = -0.497;

    // =========================================================================
    // Polynomial Angle Correction
    // =========================================================================

    /**
     * Quadratic coefficient for the polynomial correction applied to the physical angle
     * <em>before</em> the servo mapping:
     * <pre>  corrected_angle = AngleCorrA * θ² + AngleCorrB * θ + AngleCorrC</pre>
     * Default (A=0, B=1, C=0) = identity (no correction).  Use a non-zero {@code A} to
     * bow the curve if the servo response is non-linear at the extremes.
     */
    public static double AngleCorrA = 0.0;

    /**
     * Linear coefficient for the polynomial angle correction (see {@link #AngleCorrA}).
     * Default 1.0 (pass-through).  Increase to amplify small angles, decrease to compress them.
     */
    public static double AngleCorrB = 1.0;

    /**
     * Constant term for the polynomial angle correction (see {@link #AngleCorrA}).
     * Default 0.0.  Acts as a fixed angle bias in radians before the servo mapping.
     */
    public static double AngleCorrC = 0.0;

    // =========================================================================
    // Distance-Based Servo Trim
    // =========================================================================

    /**
     * Slope (servo-position units per inch of target distance) of the linear distance trim:
     * <pre>  distance_trim = DistanceTrimSlope * distance_in + DistanceTrimIntercept</pre>
     * Added on top of the physics-derived servo position after all other corrections.
     * Use when shots are consistently high/low at long range vs. short range.
     */
    public static double DistanceTrimSlope = 0.0;

    /**
     * Intercept (servo-position units) of the linear distance trim (see {@link #DistanceTrimSlope}).
     */
    public static double DistanceTrimIntercept = 0.0;

    /**
     * Zone-based distance offset table — same format as {@code RedElevationOffsetZones} in
     * {@link MDOConstants}: each row is {@code [distance_threshold_inches, servo_offset]}.
     * Rows must be sorted ascending by distance.  The last row whose threshold is ≤ the current
     * distance wins.  This is added <em>after</em> the linear trim.
     *
     * <p>Set all offsets to 0.0 to disable zone-based correction entirely.
     */
    public static double[][] ElevationZones = new double[][]{
            {0.0,   0.0},
            {100.0, 0.0},
    };

    // =========================================================================
    // Global & Alliance-Specific Trims
    // =========================================================================

    /**
     * Global additive elevation servo trim applied last (after all other corrections).
     * Use this for the final residual error once all other parameters are locked in.
     */
    public static double GlobalElevationTrim = 0.0;

    /**
     * Additional servo position trim applied only when on the Red alliance.
     * Stacks on top of {@link #GlobalElevationTrim}.
     */
    public static double RedAllianceTrim = 0.0;

    /**
     * Additional servo position trim applied only when on the Blue alliance.
     * Stacks on top of {@link #GlobalElevationTrim}.
     */
    public static double BlueAllianceTrim = 0.0;

    // =========================================================================
    // RPM Threshold & Fallback
    // =========================================================================

    /**
     * Launcher RPM below which the servo is driven to {@link #FallbackServoPosition}
     * instead of solving the physics (launcher not yet up to speed).
     */
    public static double RPMThreshold = 1000.0;

    /**
     * Servo position used when the launcher is below {@link #RPMThreshold} or when the
     * target is geometrically unreachable at the current RPM.
     * Typically the maximum-elevation position (steepest shot) so stray balls fall short
     * rather than flying past the target.
     */
    public static double FallbackServoPosition = 0.2;

    // =========================================================================
    // Servo Clamp Bounds
    // =========================================================================

    /**
     * Minimum allowed servo output position (hard lower bound applied after all corrections).
     * Prevents the servo from driving the launcher below its mechanical limit.
     */
    public static double ServoClampMin = -1.0;

    /**
     * Maximum allowed servo output position (hard upper bound applied after all corrections).
     * Prevents the servo from driving the launcher above its mechanical limit.
     */
    public static double ServoClampMax = 0.2;
}

