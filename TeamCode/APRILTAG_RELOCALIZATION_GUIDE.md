# AprilTag Relocalization Tuning Guide

## Problem
The blue alliance AprilTag relocalization wasn't working accurately, causing the robot to have incorrect position and orientation after detecting AprilTags.

## Root Cause
The AprilTag library returns the robot's pose in field coordinates, but the heading (yaw angle) uses a coordinate system that may not align with:
1. Your robot's IMU zero reference
2. Pedro Pathing's expected coordinate system
3. The field coordinate system you're using

A hardcoded 90-degree offset was being applied to ALL AprilTag detections, but this offset may be:
- Wrong for your specific setup
- Different for tags on different walls
- Different between red and blue alliances

## Solution
The fix makes the heading offset configurable via `MDOConstants.AprilTagHeadingOffset` and adds telemetry to help you tune it.

## Changes Made

### 1. MDOConstants.java
- Added `AprilTagHeadingOffset` constant (default: 90.0 degrees)
- This replaces the hardcoded offset in MainDriveOpmode.java
- You can now tune this value live using the FTC Dashboard or Configurable library

### 2. MainDriveOpmode.java
- Changed from hardcoded `Math.toRadians(90.0)` to `Math.toRadians(MDOConstants.AprilTagHeadingOffset)`
- Added telemetry to show:
  - AprilTag ID (which tag is being detected)
  - AprilTag Heading (the raw heading from the tag)
  - Decision Margin (confidence of detection)

### 3. AprilTagLocalizer.java
- Modified `getPosition()` to return 5 values instead of 4:
  - Index 0: X position (inches)
  - Index 1: Y position (inches)
  - Index 2: Z position (inches)
  - Index 3: Heading (radians)
  - Index 4: AprilTag ID (for debugging)

## How to Tune

### Step 1: Check Current Behavior
1. Run the "Drive" TeleOp
2. Position robot at a known location facing a known direction
3. Look at telemetry for "AprilTag ID" and "AprilTag Heading (rad)"
4. Note which tag ID you're detecting

### Step 2: Understand Coordinate Systems

**FTC Field Coordinate System (Standard):**
- Origin: Center of field
- X-axis: Right (positive = right side of field)
- Y-axis: Forward (positive = audience/driver station side)
- Heading: 0° faces audience wall, increases counter-clockwise

**AprilTag Coordinate System:**
- Returns robot pose relative to the tag's position on field
- Each tag has a fixed position and orientation
- The heading returned depends on the tag's orientation

### Step 3: Calculate Correct Offset

**Method A: Empirical Tuning (Recommended)**
1. Place robot facing a known direction (e.g., toward audience wall = 0°)
2. Position robot where it can see an AprilTag
3. Look at telemetry:
   - Note the "AprilTag Heading (rad)" value
   - Convert to degrees: `degrees = radians × 180 / π`
4. Calculate offset:
   ```
   Offset = Expected Heading - AprilTag Heading
   ```
5. Update `MDOConstants.AprilTagHeadingOffset` with this value
6. Test on BOTH alliances to verify it works for both

**Method B: By Tag ID**
Different tags on different walls may need different offsets:
- Red alliance observation zone tags: IDs 11, 12, 13
- Blue alliance observation zone tags: IDs 15, 16
- Submersible tags: IDs 1-10

If one alliance works but the other doesn't, you may need to:
- Check if you're detecting different tag IDs
- Verify the tag positions in the AprilTag library match your field
- Consider implementing tag-specific offsets (requires code changes)

### Step 4: Common Offset Values

| Offset | Meaning | When to Use |
|--------|---------|-------------|
| 0.0° | No rotation | AprilTag heading matches your coordinate system |
| 90.0° | Quarter turn CCW | AprilTag "forward" points to robot's right |
| -90.0° | Quarter turn CW | AprilTag "forward" points to robot's left |
| 180.0° | Half turn | AprilTag "forward" points to robot's backward |

### Step 5: Verify Decision Margin
The "Decision Margin" in telemetry indicates detection confidence:
- Values > 0.8: High confidence, relocalization will occur
- Values < 0.8: Low confidence, relocalization skipped
- If margin is consistently low, check:
  - Camera focus and calibration
  - Lighting conditions
  - Distance to AprilTag (must be < `AprilTagMaxDistance`)
  - Tag size and quality

## Testing Checklist

### Red Alliance
- [ ] Robot detects tag (ID shown in telemetry)
- [ ] Decision margin > 0.8
- [ ] After relocalization, robot X/Y position is correct
- [ ] After relocalization, robot heading is correct
- [ ] Auto-aim system points at correct target

### Blue Alliance
- [ ] Robot detects tag (ID shown in telemetry)
- [ ] Decision margin > 0.8
- [ ] After relocalization, robot X/Y position is correct
- [ ] After relocalization, robot heading is correct
- [ ] Auto-aim system points at correct target

## Troubleshooting

### Issue: "No Tag" shown in telemetry
**Causes:**
- Too far from tag (> `AprilTagMaxDistance` = 60 inches)
- Tag not in camera view
- Poor lighting
- Tag is "Obelisk" (filtered out by code)

**Solutions:**
- Move closer to tag
- Adjust camera angle
- Verify camera is working (check FTC Dashboard stream)
- Increase `AprilTagMaxDistance` if needed

### Issue: Wrong position after relocalization
**Causes:**
- `CameraOffset` is incorrect
- AprilTag library using wrong tag positions
- Camera calibration is off

**Solutions:**
- Verify `MDOConstants.CameraOffset` matches physical camera position
- Check camera mounting (pitch should be -90° for downward-facing)
- Verify you're using the correct AprilTag library for your game

### Issue: Wrong heading after relocalization
**Causes:**
- `AprilTagHeadingOffset` is incorrect
- IMU drift or miscalibration

**Solutions:**
- Follow tuning steps above
- Try offset ± 90°, ± 180° from current value
- Verify IMU is properly calibrated

### Issue: Works on red alliance but not blue
**Causes:**
- Different tags have different orientations
- Field setup is asymmetric
- Robot starting position affects which tags are visible

**Solutions:**
- Record which tag IDs you see on each alliance (check telemetry)
- Compare the "AprilTag Heading (rad)" values for red vs blue
- If they differ by exactly 180° or 90°, you may need separate offsets
- Consider implementing tag-specific offsets in code

## Advanced: Tag-Specific Offsets

If you need different offsets for different tags, modify MainDriveOpmode.java:

```java
// In the AprilTag relocalization section, replace:
follower.setPose(new Pose(aprilPose[0], aprilPose[1], 
    aprilPose[3] + Math.toRadians(MDOConstants.AprilTagHeadingOffset)));

// With:
int tagId = (int) aprilPose[4].doubleValue();
double offset = MDOConstants.AprilTagHeadingOffset;

// Adjust offset based on tag ID
if (tagId >= 11 && tagId <= 13) {
    // Red alliance tags
    offset = MDOConstants.RedAprilTagHeadingOffset;
} else if (tagId >= 15 && tagId <= 16) {
    // Blue alliance tags
    offset = MDOConstants.BlueAprilTagHeadingOffset;
}

follower.setPose(new Pose(aprilPose[0], aprilPose[1], 
    aprilPose[3] + Math.toRadians(offset)));
```

Then add to MDOConstants.java:
```java
public static double RedAprilTagHeadingOffset = 90.0;
public static double BlueAprilTagHeadingOffset = -90.0;
```

## Related Constants in MDOConstants.java

- `useAprilTags`: Enable/disable AprilTag relocalization (default: true)
- `AprilTagHeadingOffset`: Heading correction in degrees (default: 90.0)
- `AprilTagMaxDistance`: Max detection distance in inches (default: 60.0)
- `CameraOffset`: Camera position relative to robot center [x, y, z] in inches
- `AprilTagUpdateIntervalMs`: How often to process AprilTags (default: 100ms)

## Performance Notes

- AprilTag processing runs in a background thread to avoid blocking the main loop
- Detection occurs every `AprilTagUpdateIntervalMs` milliseconds
- Relocalization only happens if decision margin > 0.8
- Relocalization is rate-limited to once per 100ms to avoid jitter

## Support

If you're still having issues after following this guide:
1. Enable FTC Dashboard camera streaming to verify camera is working
2. Record telemetry values for both red and blue alliances
3. Compare tag IDs, positions, and headings between alliances
4. Check that your AprilTag library matches your game (INTO THE DEEP)
