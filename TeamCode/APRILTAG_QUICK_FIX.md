# Quick Fix for Blue AprilTag Relocalization

## Problem
Blue alliance AprilTag relocalization isn't working correctly.

## Solution
The heading offset is now configurable. Follow these steps:

## Step 1: Test Current Setup
1. Deploy the updated code to your robot
2. Run "Drive" TeleOp on **blue alliance**
3. Position robot where it can see an AprilTag
4. Check telemetry:
   - "AprilTag ID" - should show a number (not "No Tag")
   - "Decision Margin" - should be > 0.8
   - "AprilTag Heading (rad)" - note this value

## Step 2: Adjust Offset
If the robot's heading is wrong after relocalization:

### Try These Values (in order):
1. **0.0** - No offset
2. **-90.0** - Opposite of default
3. **180.0** - Reversed

### How to Change:
**Option A: FTC Dashboard (Recommended)**
- Open FTC Dashboard in browser
- Navigate to ConfigVariables
- Change `AprilTagHeadingOffset`
- Test immediately

**Option B: Code**
- Edit `TeamCode/src/main/java/.../drive/MDOConstants.java`
- Line with `AprilTagHeadingOffset = 90.0`
- Change value, rebuild, redeploy

## Step 3: Verify
Test on **BOTH** alliances:
- ✅ Robot position matches field position
- ✅ Robot heading matches actual orientation
- ✅ Auto-aim points at correct target

## Common Values

| Value | Effect |
|-------|--------|
| 90.0 | Default (AprilTag forward = robot right) |
| 0.0 | No rotation |
| -90.0 | Opposite rotation |
| 180.0 | Reversed |

## If Still Not Working

1. **Check telemetry shows "AprilTag ID"**
   - If "No Tag": Move closer, check camera
   
2. **Check "Decision Margin" > 0.8**
   - If too low: Move closer, improve lighting
   
3. **Different tags on red vs blue?**
   - Record AprilTag ID on each alliance
   - May need tag-specific offsets (see full guide)

## Full Documentation
See `APRILTAG_RELOCALIZATION_GUIDE.md` for complete tuning instructions.

## Files Changed
- ✅ MDOConstants.java - Added `AprilTagHeadingOffset`
- ✅ MainDriveOpmode.java - Uses configurable offset, added telemetry
- ✅ AprilTagLocalizer.java - Returns AprilTag ID

## Support
If you need help:
1. Note which AprilTag IDs you see on red vs blue
2. Note the "AprilTag Heading (rad)" values
3. Check if heading error is consistent (always off by same amount)
