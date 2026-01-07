# TeamCode TODO List

## RobotCoreCustom.java Issues

### Critical Errors
NONE

### Code Quality Issues (Warnings)

#### Unused Parameters
- **Line 28**: Parameter `follower` in constructor `RobotCoreCustom(HardwareMap hardwareMap, Follower follower)` is never used
  - **Fix**: Either use the parameter or remove it from the signature

#### Unused Methods
- **Line 33**: Method `getExternalHeading()` is never used
  - **Fix**: Verify if this method is needed; if not, consider removing it or adding a TODO comment explaining its purpose

#### Unused Constructors
- **Line 87**: Constructor `CustomMotor(HardwareMap hardwareMap, String motorName)` is never used
  - **Fix**: Either use this constructor or remove it; consider if the full constructor with encoder support should be the only one

#### Unused Fields
- **Line 204**: Private field `customColor0` is never used
- **Line 207**: Private field `customColor1` is never used
- **Line 208**: Private field `customColor2` is never used
  - **Fix**: Remove these unused fields or add documentation explaining their purpose

#### Unused Methods
- **Line 312**: Method `setSolidColor(int, int, int, int, int)` in `CustomRGBController` is never used
  - **Fix**: Either use this method or remove it; commented code suggests it was replaced by `setThird()`

#### Unused Return Values
- **Line 322**: Return value of method `setThird()` is never used
  - **Fix**: Either use the return value or change it to `void` if error handling is not needed

#### Array Declaration Style Issues
- **Line 205**: `int lifterState[] = new int[3];` - Uses C-style array declaration
  - **Fix**: Change to `int[] lifterState = new int[3];`

- **Line 209**: `private Servo lifter[] = new Servo[3];` - Uses C-style array declaration
  - **Fix**: Change to `private Servo[] lifter = new Servo[3];`

- **Line 210**: `private ColorSensor colorSensor[] = new ColorSensor[6];` - Uses C-style array declaration
  - **Fix**: Change to `private ColorSensor[] colorSensor = new ColorSensor[6];`

#### Fields That Should Be Final
- **Line 209**: Field `lifter` should be declared `final`
  - **Fix**: Change to `private final Servo[] lifter = new Servo[3];`

- **Line 210**: Field `colorSensor` should be declared `final`
  - **Fix**: Change to `private final ColorSensor[] colorSensor = new ColorSensor[6];`

- **Line 211**: Field `RGBPrism` should be declared `final`
  - **Fix**: Change to `private final CustomRGBController RGBPrism;` (initialize in constructor)

---

## Tuning.java Issues

### Unused Imports (Lines 10-40)
The following imports are declared but not used in the code. Consider removing them:
- Line 10: `androidx.annotation.NonNull`
- Line 15: `com.bylazar.field.FieldManager`
- Line 16: `com.bylazar.field.ImagePreset`
- Line 17: `com.bylazar.field.PanelsField`
- Line 18: `com.bylazar.field.Style`
- Line 29: `com.bylazar.configurables.annotations.Configurable`
- Line 30: `com.bylazar.configurables.annotations.IgnoreConfigurable`
- Line 31: `com.bylazar.configurables.PanelsConfigurables`
- Line 32: `com.bylazar.field.FieldManager` (duplicate)
- Line 33: `com.bylazar.field.PanelsField` (duplicate)
- Line 34: `com.bylazar.field.Style` (duplicate)
- Line 35: `com.bylazar.telemetry.PanelsTelemetry`
- Line 36: `com.bylazar.telemetry.TelemetryManager`
- Line 40: `java.util.UUID`

**Fix**: Remove these unused imports or if they're needed for future development, add a comment explaining why they're imported.

---

## Summary

**Total Issues Found: 23**
- Critical Errors: 1 (Array index out of bounds)
- Code Quality Warnings: 22

### Priority Fixes
1. **High Priority**: Fix array index out of bounds in `CustomRGBController.setThird()` - This will cause runtime crashes
2. **Medium Priority**: Clean up unused fields and methods in `RobotCoreCustom`
3. **Low Priority**: Fix array declaration style and add final modifiers for code quality
4. **Low Priority**: Remove unused imports from `Tuning.java`


