# AprilTag Relocalization Testing Checklist

## Pre-Test Setup
- [ ] Code deployed to robot
- [ ] FTC Dashboard connected and accessible
- [ ] Robot charged and ready
- [ ] Field AprilTags in place and visible

## Initial Configuration Check
Current values in MDOConstants.java:
- `useAprilTags` = true
- `AprilTagHeadingOffset` = 90.0 degrees
- `AprilTagMaxDistance` = 60.0 inches
- `CameraOffset` = [-4.2, -7.5, -12.7] inches

## Test 1: Red Alliance Baseline

### Setup
1. Place robot on RED alliance side
2. Position robot 30-40 inches from a red alliance AprilTag
3. Face robot toward the tag (or at known angle)
4. Start "Drive" TeleOp

### Expected Telemetry
- [ ] "AprilTag ID" shows a number (e.g., 11, 12, or 13 for red)
- [ ] "Decision Margin" > 0.8
- [ ] "AprilTag Heading (rad)" shows a value
- [ ] "X (in) AprilTag" and "Y (in) AprilTag" show coordinates

### Verification
- [ ] Robot position on field matches telemetry X/Y values
- [ ] Robot heading after relocalization matches actual orientation
- [ ] Auto-aim system points at red target correctly
- [ ] Record AprilTag ID detected: _______
- [ ] Record AprilTag Heading (rad): _______

### Notes:
```
[Space for observations]
```

---

## Test 2: Blue Alliance Initial Test

### Setup
1. Place robot on BLUE alliance side
2. Position robot 30-40 inches from a blue alliance AprilTag
3. Face robot in SAME orientation as red alliance test
4. Start "Drive" TeleOp

### Expected Telemetry
- [ ] "AprilTag ID" shows a number (e.g., 15, 16 for blue)
- [ ] "Decision Margin" > 0.8
- [ ] "AprilTag Heading (rad)" shows a value
- [ ] "X (in) AprilTag" and "Y (in) AprilTag" show coordinates

### Verification
- [ ] Robot position on field matches telemetry X/Y values
- [ ] Robot heading after relocalization matches actual orientation
- [ ] Auto-aim system points at blue target correctly
- [ ] Record AprilTag ID detected: _______
- [ ] Record AprilTag Heading (rad): _______

### Issue Detection
**If heading is WRONG on blue but RIGHT on red:**
- [ ] Calculate difference between red and blue heading values
- [ ] Try adjusting `AprilTagHeadingOffset` by this difference
- [ ] Proceed to Test 3

**If heading is WRONG on both:**
- [ ] Try `AprilTagHeadingOffset = 0.0`
- [ ] If still wrong, try `-90.0` or `180.0`
- [ ] Proceed to Test 3

### Notes:
```
[Space for observations]
```

---

## Test 3: Tuning AprilTagHeadingOffset

### Iteration 1: Try Alternative Offset

**Value to test:** _______ degrees

1. Open FTC Dashboard → ConfigVariables
2. Change `AprilTagHeadingOffset` to test value
3. **Blue alliance:** Position robot, verify heading
4. **Red alliance:** Position robot, verify heading

**Results:**
- [ ] Red heading: CORRECT / WRONG
- [ ] Blue heading: CORRECT / WRONG

### Iteration 2: Fine Tuning (if needed)

**Value to test:** _______ degrees

1. Adjust offset by small increments (±5 to ±10 degrees)
2. Test both alliances
3. Record results

**Results:**
- [ ] Red heading: CORRECT / WRONG
- [ ] Blue heading: CORRECT / WRONG

### Final Working Value
**AprilTagHeadingOffset = _______ degrees**

---

## Test 4: Distance and Confidence Testing

### Close Range (20-30 inches)
- [ ] Red alliance: Decision Margin = _______
- [ ] Blue alliance: Decision Margin = _______
- [ ] Both > 0.8: PASS / FAIL

### Medium Range (40-50 inches)
- [ ] Red alliance: Decision Margin = _______
- [ ] Blue alliance: Decision Margin = _______
- [ ] Both > 0.8: PASS / FAIL

### Maximum Range (60+ inches)
- [ ] Red alliance: Decision Margin = _______
- [ ] Blue alliance: Decision Margin = _______
- [ ] Note: May need to adjust `AprilTagMaxDistance`

---

## Test 5: Multiple Tags

### Red Alliance
**Tag 1:** ID = _______
- [ ] Position correct
- [ ] Heading correct

**Tag 2:** ID = _______
- [ ] Position correct
- [ ] Heading correct

**Tag 3:** ID = _______
- [ ] Position correct
- [ ] Heading correct

### Blue Alliance
**Tag 1:** ID = _______
- [ ] Position correct
- [ ] Heading correct

**Tag 2:** ID = _______
- [ ] Position correct
- [ ] Heading correct

**Tag 3:** ID = _______
- [ ] Position correct
- [ ] Heading correct

---

## Test 6: Auto-Aim Verification

### Red Alliance
1. Position robot at random field location
2. Wait for AprilTag relocalization (Decision Margin > 0.8)
3. Enable auto-aim (launcher spinning)
4. [ ] Azimuth servo points toward red target
5. [ ] Elevation servo angle looks correct
6. [ ] Fire test shot → Ball lands near target

### Blue Alliance
1. Position robot at random field location
2. Wait for AprilTag relocalization (Decision Margin > 0.8)
3. Enable auto-aim (launcher spinning)
4. [ ] Azimuth servo points toward blue target
5. [ ] Elevation servo angle looks correct
6. [ ] Fire test shot → Ball lands near target

---

## Test 7: Edge Cases

### Robot at 45-degree angle
- [ ] Red alliance: Relocalization works
- [ ] Blue alliance: Relocalization works

### Robot very close (< 20 inches)
- [ ] Red alliance: Relocalization works
- [ ] Blue alliance: Relocalization works

### Robot at maximum distance
- [ ] Red alliance: Relocalization works
- [ ] Blue alliance: Relocalization works

### Moving robot during detection
- [ ] Red alliance: No false relocalization
- [ ] Blue alliance: No false relocalization

---

## Final Verification

### System Integration
- [ ] Relocalization doesn't cause drive jitter
- [ ] Field-centric drive works correctly after relocalization
- [ ] No performance issues (loop time < 20ms)
- [ ] No thermal throttling warnings

### Documentation
- [ ] Final `AprilTagHeadingOffset` value recorded: _______
- [ ] Any tag-specific issues noted in documentation
- [ ] Camera offset verified: _______
- [ ] IMU orientation verified (Logo UP, USB LEFT)

---

## Troubleshooting Results

### Issues Encountered
```
[Describe any issues found during testing]
```

### Solutions Applied
```
[Describe what fixed the issues]
```

### Outstanding Issues
```
[List any unresolved problems]
```

---

## Sign-Off

**Tested by:** _______________________  
**Date:** _______________________  
**Final Status:** PASS / FAIL / NEEDS MORE WORK

**Red Alliance:** ✅ / ❌  
**Blue Alliance:** ✅ / ❌  
**Auto-Aim:** ✅ / ❌  
**Performance:** ✅ / ❌

**Notes:**
```
[Any additional observations or recommendations]
```

---

## Quick Reference

### Telemetry Values to Monitor
- **AprilTag ID**: Which tag is detected
- **Decision Margin**: Detection confidence (need > 0.8)
- **AprilTag Heading (rad)**: Raw heading from tag
- **Pose X/Y**: Robot position in inches

### Common Offset Values
| Offset | Use Case |
|--------|----------|
| 0.0° | No correction needed |
| 90.0° | Default (AprilTag forward = robot right) |
| -90.0° | Reverse of default |
| 180.0° | Backward facing correction |

### FTC Dashboard Path
1. Connect to robot WiFi
2. Open browser: http://192.168.43.1:8080/dash
3. Navigate to: ConfigVariables → MDOConstants
4. Edit: `AprilTagHeadingOffset`
5. Changes apply immediately (no rebuild needed)
