# Aiming System Implementation Guide

## Overview
A comprehensive aiming system has been added to MainDriveOpmode with extensive tuning capabilities through MDOConstants. The system supports both elevation and azimuth control with smooth tracking, manual adjustments, and field-relative aiming.

## Features

### 1. **Auto-Aiming**
- Automatically calculates elevation and azimuth angles to hit a target location
- Uses `RobotCoreCustom.localizerLauncherCalc()` for ballistic trajectory calculations
- Supports both robot-relative and field-relative aiming modes

### 2. **Dual Servo Support**
- **Elevation Servo**: Controls vertical angle (pitch)
- **Azimuth Servos**: Controls horizontal angle (yaw)
  - Supports single or dual servo configuration
  - Dual servos can be mirrored for synchronized movement

### 3. **Manual Adjustments**
- **Gamepad2 D-Pad Up/Down**: Adjust elevation offset
- **Gamepad2 D-Pad Left/Right**: Adjust azimuth offset
- **Gamepad2 Y Button**: Reset all manual offsets to zero
- Adjustments are cumulative and work with auto-aiming

### 4. **Smoothing & Stability**
- Configurable smoothing factor to reduce jitter
- Deadzone settings to ignore micro-adjustments
- Prevents oscillation and provides stable aiming

### 5. **Aim Lock Detection**
- Tracks whether the aim is within acceptable error margins
- Reports lock status to telemetry
- Can be used to prevent shooting until aim is locked (future enhancement)

## MDOConstants Configuration

### Basic Enable/Disable
```java
public static boolean enableAutoAiming = true;  // Master switch for aiming system
```

### Elevation Servo Configuration
```java
// Servo position limits (0.0 to 1.0)
public static double elevationServoMin = 0.0;
public static double elevationServoMax = 1.0;

// Angle limits (radians)
public static double elevationAngleMin = 0.0;           // 0 degrees
public static double elevationAngleMax = Math.toRadians(45.0); // 45 degrees

// Calibration
public static double elevationServoOffset = 0.0;        // Add offset if servo isn't centered
public static boolean elevationServoReversed = false;   // Flip servo direction if needed
```

### Azimuth Servo Configuration
```java
// Servo position limits (0.0 to 1.0)
public static double azimuthServoMin = 0.0;
public static double azimuthServoMax = 1.0;

// Angle limits (radians)
public static double azimuthAngleMin = -Math.PI;        // -180 degrees
public static double azimuthAngleMax = Math.PI;         // +180 degrees

// Calibration
public static double azimuthServoOffset = 0.0;          // Add offset if servo isn't centered
public static boolean azimuthServoReversed = false;     // Flip servo direction if needed
public static boolean useFieldRelativeAzimuth = true;   // Compensate for robot rotation
```

### Dual Servo Configuration
```java
public static boolean useDualAzimuthServos = true;      // Enable second azimuth servo
public static boolean azimuthServo1Mirrored = true;     // Mirror second servo movement
```

### Smoothing & Deadzone
```java
// Smoothing: 0.0 = no movement, 1.0 = instant response, 0.2 = smooth tracking
public static double aimingSmoothingFactor = 0.2;

// Deadzone: Ignore changes smaller than this (radians)
public static double elevationDeadzone = Math.toRadians(1.0);  // 1 degree
public static double azimuthDeadzone = Math.toRadians(2.0);    // 2 degrees
```

### Manual Adjustment Settings
```java
// Step size per button press (radians)
public static double manualElevationStep = Math.toRadians(1.0);  // 1 degree
public static double manualAzimuthStep = Math.toRadians(2.0);    // 2 degrees

// Time between adjustments when button held (milliseconds)
public static double manualAdjustmentHoldTime = 100.0;
```

### Aim Lock Configuration
```java
// Maximum acceptable error for "aim locked" status (radians)
public static double maxElevationError = Math.toRadians(5.0);  // 5 degrees
public static double maxAzimuthError = Math.toRadians(5.0);    // 5 degrees

// Future: Prevent shooting until aim is locked
public static boolean requireAimLock = false;  // Not yet implemented
```

## Telemetry Output

When aiming is enabled, the following telemetry is displayed:

```
--- Aiming System ---
Target Elevation (deg): 25.3
Current Elevation (deg): 24.8
Target Azimuth (deg): 15.7
Current Azimuth (deg): 16.2
Manual Elev Offset (deg): 0.0
Manual Azim Offset (deg): 0.0
Aim Locked: YES
```

## Hardware Configuration

### Required Servo Names
The system expects the following servo names in the hardware map:
- `elevationServo` - Controls elevation (vertical angle)
- `azimuthServo0` - Primary azimuth servo (horizontal angle)
- `azimuthServo1` - Secondary azimuth servo (optional, if useDualAzimuthServos = true)

### Target Location
The target location is set based on team color:
- **RED Team**: `{-70.0, -70.0, 40.0}` (x, y, z in inches)
- **BLUE Team**: `{-70.0, 70.0, 40.0}` (x, y, z in inches)

## Tuning Guide

### Step 1: Basic Servo Setup
1. Set `enableAutoAiming = false` temporarily
2. Manually set servo positions to verify direction:
   - If servo moves opposite direction, set `xxxServoReversed = true`
3. Measure physical angle limits and update `xxxAngleMin/Max`

### Step 2: Calibration
1. Enable auto-aiming: `enableAutoAiming = true`
2. Drive robot to known position
3. Check telemetry for target vs current angles
4. Adjust `xxxServoOffset` if servo center doesn't match expected position

### Step 3: Smoothing
1. Start with `aimingSmoothingFactor = 0.2` (default)
2. If aiming is too sluggish, increase to 0.3-0.5
3. If aiming is jittery, decrease to 0.1-0.15

### Step 4: Deadzone
1. Watch telemetry for small oscillations
2. If aiming "hunts" back and forth, increase deadzone values
3. Typical values: 0.5-2.0 degrees (0.009-0.035 radians)

### Step 5: Manual Adjustments
1. Test manual controls with D-pad
2. Adjust step sizes if adjustments are too coarse/fine
3. Adjust hold time if repeated adjustments are too fast/slow

## How It Works

1. **Calculate Target Angles**: Uses launcher position, target position, and physics to calculate required elevation and azimuth
2. **Apply Manual Offsets**: Adds any manual adjustments from gamepad
3. **Clamp to Limits**: Ensures angles stay within configured min/max ranges
4. **Apply Smoothing**: Gradually moves current angle toward target using smoothing factor
5. **Check Deadzone**: Ignores tiny changes to prevent jitter
6. **Convert to Servo Position**: Maps angle (radians) to servo position (0.0-1.0)
7. **Apply Calibration**: Adds offset and reverses if needed
8. **Set Servos**: Updates servo positions

## Field-Relative vs Robot-Relative

### Robot-Relative (`useFieldRelativeAzimuth = false`)
- Azimuth is relative to robot's front
- Easier to visualize during testing
- Target appears to move as robot rotates

### Field-Relative (`useFieldRelativeAzimuth = true`)
- Azimuth compensates for robot heading
- Target stays fixed on field
- Better for moving while aiming

## Future Enhancements

- Implement `requireAimLock` to prevent shooting until aimed
- Add velocity compensation for moving targets
- Implement elevation/azimuth limits based on physical constraints
- Add visual feedback (LED indicators) for aim lock status
- Log aim accuracy for post-match analysis

