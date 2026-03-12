# AGENTS.md

## Project Overview

FTC (FIRST Tech Challenge) robot controller for the **DECODE 2025-2026** season. Java/Android project built on the FTC SDK v11.0, using **Pedro Pathing** for path following and **SolversLib** for utilities. The robot features a ball launcher with auto-aiming turret, intake, color-sorting system, and mecanum drivetrain with two-wheel odometry.

## Architecture

### Module Layout
- **`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`** — All team code lives here
  - `drive/opmode/` — Runnable OpModes (`MainDriveOpmode`, `GamepadEventDemo`)
  - `drive/auto/` — Autonomous OpModes + paired `*Constants.java` files (one per alliance/position)
  - `drive/` — `AprilTagLocalizer`, `AutoToTeleDataTransferer` (singleton for auto→teleop data persistence)
  - `modules/` — Subsystems and hardware abstractions (the core of the robot logic)
  - `constants/MDOConstants.java` — **Central config file** annotated with `@Config` for FTC Dashboard live-tuning
  - `pedroPathing/Constants.java` — Drivetrain/follower/localizer tuning constants

### Subsystem Pattern
Each mechanism is a standalone subsystem class in `modules/`:
- `TurretSubsystem` — Threaded turret aiming (~500-1000Hz lock-free loop via volatile fields)
- `ElevationSubsystem` — Servo-based launch elevation with distance-zone offsets
- `LauncherSubsystem` — Motor speed control, rapid-fire sequencing, PID/manual modes
- `IntakeSubsystem` — IN/OUT/STOP state machine with auto-stop at 3 balls
- `CustomSorterController` — 6 color sensors + 3 lifter servos for ball sorting by color (green/purple)

Subsystems receive inputs via setter methods and are updated each loop. The main OpMode (`MainDriveOpmode`) orchestrates them.

### Threading Model
`CustomThreads` manages background threads for: servo PID loops, follower updates, AprilTag processing, sorter sensor reads, CPU monitoring, and FTC Dashboard drawing. Shared state uses `volatile` fields and `synchronized` blocks (lock-free for hot paths). The `TurretSubsystem` runs its own dedicated aiming thread.

### Gamepad Input
`GamepadEventHandler` is a lambda-driven event system. Bindings are created with `bindPress`/`bindDebouncedPress`/`bindAnalogPress`, then actions are tagged and attached. See its Javadoc for usage examples.

## Build & Deploy

```bash
# Build APK (from project root)
./gradlew assembleDebug

# Build and install to connected device
./gradlew installDebug
```

- Requires **Android Studio Ladybug (2024.2)+** and a connected REV Control Hub or Android device
- AGP 8.7.0, compileSdk 34, minSdk 24
- External repos: `maven.pedropathing.com`, `repo.dairy.foundation/releases`

## Key Conventions

- **`@Config` classes** (`MDOConstants`, `BlueCloseAutoConstants`, etc.) are live-tunable via FTC Dashboard — all `public static` fields are exposed. Never make these fields `final`.
- **Alliance-specific constants** are prefixed `Red`/`Blue` in `MDOConstants` (e.g., `RedLauncherRPM`, `BlueLauncherCalcConstants`). Auto opmodes come in `Red*`/`Blue*` + `Close`/`Far` variants.
- **Auto OpMode pattern**: Each auto has a paired `*Constants.java` with `@Config` for poses/speeds/timings, and an `*Opmode.java` with a state-machine `enum AutoState`. States advance via `follower.isBusy()` checks and timers.
- **Hardware names** are string literals in constructors (e.g., `"launcher0"`, `"azimuthServo0"`, `"colorSensor0"`). These must match the robot's configuration file on the Control Hub.
- **Custom wrappers** (`CustomMotor`, `CustomMotorController`, `CustomServoController`) abstract over FTC SDK hardware with added PID, grouping, encoder support, and thread safety. Always use these instead of raw SDK classes.
- **Auto→TeleOp handoff** uses `AutoToTeleDataTransferer` singleton to persist pose, alliance, and custom data across OpMode transitions.
- **Bulk caching** via `HubInitializer.initBulkCaching()` is called in every OpMode `init()` to reduce I2C overhead.

## Dependencies (TeamCode)

| Library | Purpose |
|---------|---------|
| `com.pedropathing:ftc:2.0.3` | Path following + mecanum drive |
| `org.solverslib:core:0.3.4` | PIDFController, utilities |
| `org.solverslib:pedroPathing:0.3.4` | SolversLib Pedro integration |
| `org.solverslib:photon:0.3.4` | Performance optimizations |
| `dev.frozenmilk.sinister:Sloth:0.2.4` | Annotation-based OpMode loading |
| `com.acmerobotics.slothboard:dashboard` | FTC Dashboard for live tuning |

## Other Components

- **`Python Ball Tracer/`** — Pygame projectile trajectory simulator for tuning launcher physics
- **`wiring-diagram/`** — KiCad hardware wiring schematics

