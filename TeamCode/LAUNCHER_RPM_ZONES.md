# Launcher RPM Zones Feature

## Overview
Added dynamic RPM zones that automatically adjust launcher motor speed based on the robot's distance to the target. This allows for more accurate shooting at different ranges.

## How It Works

### 1. Distance Calculation
The system calculates the 3D distance from the robot's current position to the target (red or blue basket) using:
- Robot position from the Pedro Pathing follower
- Target position from `MDOConstants.redTargetLocation` or `MDOConstants.blueTargetLocation`
- `LocalizationAutoAim.getDistance()` method

### 2. RPM Zone Selection
Based on the calculated distance, the system selects the appropriate RPM from configured zones:

| Distance Range | Default RPM | Use Case |
|---------------|-------------|----------|
| 0-30 inches | 1800 | Close range, lower power needed |
| 30-60 inches | 2000 | Medium-close range |
| 60-90 inches | 2300 | Medium range (default) |
| 90-120 inches | 2600 | Medium-far range |
| 120+ inches | 3000 | Long range, maximum power |

### 3. Implementation Details

**Files Modified:**
- `MDOConstants.java` - Added zone configuration
- `RobotCoreCustom.java` - Added `calculateLauncherRPM()` method
- `MainDriveOpmode.java` - Integrated dynamic RPM calculation

**Key Methods:**
- `RobotCoreCustom.calculateLauncherRPM(Follower follower, Double[] target)` - Returns appropriate RPM based on distance
- Algorithm searches zones from highest to lowest distance threshold for efficiency

## Configuration

### Adjusting RPM Zones
Edit the `LauncherRPMZones` array in `MDOConstants.java`:

```java
public static double[][] LauncherRPMZones = new double[][]{
    {0.0, 1800},    // Distance threshold, RPM value
    {30.0, 2000},
    {60.0, 2300},
    {90.0, 2600},
    {120.0, 3000},
};
```

**Format:** `{distance_threshold_inches, rpm_value}`
- Distance thresholds are minimum values for that zone
- Zones are checked from highest to lowest
- First matching zone is used

### Enable/Disable Feature
```java
public static boolean EnableLauncherRPMZones = true;  // Set to false to use static RPM
```

When disabled, the system uses the static `LauncherRPM` value (2300).

## Telemetry
The following data is displayed on the driver station:
- **Distance to Target (in)** - Current distance in inches
- **Dynamic RPM (Target)** - The calculated target RPM for current distance
- **Launcher RPM** - Actual current RPM from motors

## Tuning Tips

1. **Test at each distance range** - Drive to different positions and check if balls reach target
2. **Adjust RPM values** - If overshooting, reduce RPM; if undershooting, increase RPM
3. **Fine-tune zones** - Adjust distance thresholds if transitions feel abrupt
4. **Consider field conditions** - Battery voltage and friction may require adjustments

## Benefits

✅ **Automatic adjustment** - No manual RPM changes needed
✅ **Better accuracy** - Optimized power for each distance
✅ **Battery efficiency** - Don't use max power when not needed
✅ **Easy tuning** - All values in one place (MDOConstants)
✅ **Toggle on/off** - Can disable if needed without code changes

## Future Enhancements (Optional)

- [ ] Add RPM interpolation between zones for smoother transitions
- [ ] Account for robot velocity (moving vs stationary shots)
- [ ] Battery voltage compensation
- [ ] Different zone profiles for different shooting strategies
