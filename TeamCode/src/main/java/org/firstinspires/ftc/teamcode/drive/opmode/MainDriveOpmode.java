package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.drive.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.drive.AutoToTeleDataTransferer;
import org.firstinspires.ftc.teamcode.drive.CustomPIDFController;
import org.firstinspires.ftc.teamcode.drive.CustomThreads;
import org.firstinspires.ftc.teamcode.drive.MDOConstants;
import org.firstinspires.ftc.teamcode.drive.RobotCoreCustom;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Drive", group = "!advanced")
public class MainDriveOpmode extends OpMode {
    AprilTagLocalizer localizer;
    //IMU imuEX;
    RobotCoreCustom robotCoreCustom;
    Follower follower;
    RobotCoreCustom.CustomAxonServoController elevationServo;
    RobotCoreCustom.CustomAxonServoController azimuthServo;
    Double launchElevationDeg = 0.0;
    double elevationServoTarget = 0.0;
    RobotCoreCustom.CustomMotorController launcherMotors, intakeMotor;
    RobotCoreCustom.CustomMotor drawbridgeMotor;
    Double[] lastAprilLocalization = null;
    double targetPower = 0.0;
    double elevationServoFinal = 0;
    ElapsedTime gamepadTimer = new ElapsedTime();
    RobotCoreCustom.CustomTelemetry telemetryC;
    RobotCoreCustom.CustomSorterController sorterController;
    boolean launcherSpinning = false;
    private Double[] launchVectors;
    int prevBallCount = 0;

    enum Team {
        RED,
        BLUE
    }

    Team team = Team.BLUE;
     ElapsedTime lifterAutoLaunchTimer = new ElapsedTime();
     ElapsedTime aprilSlowdownTimer = new ElapsedTime();
     ElapsedTime loopTimer = new ElapsedTime();
     ElapsedTime intakeInputTimer = new ElapsedTime();
    double loopTimeMs = 0;
    ElapsedTime sectionTimer = new ElapsedTime();
    double timeCaching = 0, timeUpdate = 0, timeSorter = 0, timeDrive = 0;
    double timeAprilTag = 0, timeServo = 0, timeLauncher = 0, timeIntake = 0, timeTelemetry = 0;

    Double launchAzimuthDeg = null;
    Double fieldRelativeAzimuthDeg;
    Double robotFieldRelativeAzimuthDeg;
    Double finalAzimuthDeg;
    static TelemetryManager telemetryM;
    enum intakeState {
        IN,
        OUT,
        STOP
    }
    intakeState intakeRunningState = intakeState.STOP;

    // Telemetry throttling to reduce overhead
    private int telemetryLoopCounter = 0;
    private static final int TELEMETRY_UPDATE_INTERVAL = 5; // Update telemetry every N loops (higher = less lag)

    // Threads
    // Handles background tasks like sensor reading and servo PID to keep the main loop fast
    CustomThreads customThreads;

	@Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryC = new RobotCoreCustom.CustomTelemetry(telemetry, telemetryM);

        // Sorting Initialization
        // Initializes servos and color sensors for the ball sorting mechanism
        sorterController = new RobotCoreCustom.CustomSorterController(hardwareMap);

        // Get data from autonomous if available
        AutoToTeleDataTransferer dataTransfer = AutoToTeleDataTransferer.getInstance();
        Pose startPose = dataTransfer.getEndPose();
        
        robotCoreCustom = new RobotCoreCustom(hardwareMap, follower);

        localizer = new AprilTagLocalizer();

        // Apply initial value
        localizer.cameraPosition = new Position(DistanceUnit.INCH,
                MDOConstants.CameraOffset[0], MDOConstants.CameraOffset[1], MDOConstants.CameraOffset[2], 0);

        elevationServo = new RobotCoreCustom.CustomAxonServoController(
                hardwareMap,
                new String[]{"elevationServo"},
                new boolean[]{false},
                false,
                new double[]{0, 0, 0},
                null
        );
        azimuthServo = new RobotCoreCustom.CustomAxonServoController(
                hardwareMap,
                new String[] {"azimuthServo0", "azimuthServo1"},
                new boolean[] {true, true},
                true,
                MDOConstants.AzimuthPIDFConstants,
                "azimuthPosition"
        );
        localizer.initAprilTag(hardwareMap, "Webcam 1");

        launcherMotors = new RobotCoreCustom.CustomMotorController(
                hardwareMap,
                new String[]{"launcher0", "launcher1"},
                new boolean[]{false, true}, // launcher1 is reversed
                new boolean[]{false, true}, // encoder reverse map: launcher1 encoder is reversed
                true, // has encoders
                28.0, // ticks per rev
                new CustomPIDFController(0, 0, 0, 0)
        );
        intakeMotor = new RobotCoreCustom.CustomMotorController(
                hardwareMap,
                new String[]{"intake"},
                new boolean[]{true},
                false,
                28.0,
                new CustomPIDFController(0, 0, 0, 0)
        );
        drawbridgeMotor = new RobotCoreCustom.CustomMotor(hardwareMap, "drawBridge", false, 67, new CustomPIDFController(0, 0, 0, 0));

        follower = Constants.createFollower(hardwareMap);

        // Use pose from auto if available, otherwise use default
        if (startPose.getX() != 0 || startPose.getY() != 0 || startPose.getHeading() != 0) {
            follower.setStartingPose(startPose);
            telemetryC.addData("Starting Pose from Auto",
                String.format("(%.1f, %.1f, %.1f°)",
                    startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading())));
        } else {
            follower.setStartingPose(new Pose(0, 0, 0));
            telemetryC.addData("Starting Pose", "Default (0, 0, 0)");
        }
        telemetryC.update();

        follower.startTeleopDrive();
        gamepadTimer.reset();
        aprilSlowdownTimer.reset();
        lifterAutoLaunchTimer.reset();
        intakeInputTimer.reset();
        customThreads = new CustomThreads(robotCoreCustom, follower);
        customThreads.setAzimuthServo(azimuthServo);
        customThreads.setLauncherMotors(launcherMotors);
        // Pass the sorter controller to the thread manager so it can update sensors in the background
        customThreads.setSorterController(sorterController);
        // Pass the AprilTag localizer to process detections in background
        customThreads.setAprilTagLocalizer(localizer);

    }

    @Override
    public void init_loop() {
        if (gamepad1.x) {
            team = Team.BLUE;
        } else if (gamepad1.b) {
            team = Team.RED;
        }

        telemetryC.addData("Selected Team", team);
        telemetryC.addData("Select Team", "Press X for Blue, B for Red");
        telemetryC.update();
    }

    @Override
    public void start() {
        customThreads.startDrawingThread();
        customThreads.startCPUMonThread();
        customThreads.startAzimuthPIDThread();
        // Start the thread that reads color sensors ~20 times/second
        customThreads.startSorterThread();
        // Start the drive thread for more responsive driving
        if (MDOConstants.EnableThreadedDrive) {
            customThreads.startDriveThread();
        }
        // Start the AprilTag processing thread to offload vision processing
        customThreads.startAprilTagThread();
        // Start the launcher PIDF thread for RPM control
        customThreads.startLauncherPIDThread();
    }
    @Override
    public void stop() {
        customThreads.stopDrawingThread();
        customThreads.stopCPUMonThread();
        customThreads.stopAzimuthPIDThread();
        // Stop the thread to prevent resource leaks
        customThreads.stopSorterThread();
        // Stop the drive thread
        if (MDOConstants.EnableThreadedDrive) {
            customThreads.stopDriveThread();
        }
        // Stop the AprilTag thread
        customThreads.stopAprilTagThread();
        // Stop the launcher PID thread
        customThreads.stopLauncherPIDThread();
        localizer.stopStream();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        loopTimer.reset();

        // ===== CACHE ALL VALUES AT THE BEGINNING =====
        sectionTimer.reset();

        // Follower/Pose data
        Pose currentPose;
        double poseX, poseY, robotHeadingRad;
        currentPose = follower.getPose();
        poseX = currentPose.getX();
        poseY = currentPose.getY();
        robotHeadingRad = follower.getHeading();
        robotFieldRelativeAzimuthDeg = Math.toDegrees(robotHeadingRad);


        // Launch calculations
        launchVectors = RobotCoreCustom.localizerLauncherCalc(follower, (team == Team.RED) ?
                MDOConstants.redTargetLocation : MDOConstants.blueTargetLocation);

        // Launcher data
        double launcherRPM = launcherMotors.getAverageRPM();

        // Gamepad inputs - all sticks
        double gamepad1LeftStickY = gamepad1.left_stick_y;
        double gamepad1LeftStickX = gamepad1.left_stick_x;
        double gamepad1RightStickX = gamepad1.right_stick_x;

        // Timer values
        double gamepadTimerMs = gamepadTimer.milliseconds();

        // Cache azimuth servo data ONCE to avoid hardware I/O in telemetry
        double servoPosition = azimuthServo.getPosition();
        double servoTarget = azimuthServo.getTargetPosition();

        // Pre-format CPU strings to avoid repeated String.format() in telemetry (expensive operation)
        double cpuUsage = customThreads.getCpuUsage();
        double cpuTemp = customThreads.getCpuTemp();

        // Cache AprilTag position from the background thread (non-blocking)
        Double[] aprilPose = customThreads.getCachedAprilPose();
        timeCaching = sectionTimer.milliseconds();

        // ===== MAIN LOOP LOGIC =====
        sectionTimer.reset();

        follower.update();
        timeUpdate = sectionTimer.milliseconds();

        // ===== SORTER CONTROLLER UPDATES =====
        sectionTimer.reset();
        sorterController.lifterUpdater();
        //sorterController.lightingUpdater();

        // Ball Counting
        // Retrieve the latest ball count from the background thread's cache.
        // This is non-blocking and instant, unlike direct I2C sensor reads.
        int currentBallCount = sorterController.getCachedBallCount();

        timeSorter = sectionTimer.milliseconds();
        // azimuthServo.servoPidLoop() now runs in separate thread

        // ===== DRIVE CONTROL =====
        sectionTimer.reset();

        if (MDOConstants.EnableThreadedDrive) {
            // Use threaded drive for maximum responsiveness
            // The drive thread runs at ~200Hz independently of this loop
            customThreads.setDriveInputs(
                    -gamepad1LeftStickY,
                    -gamepad1LeftStickX,
                    gamepad1RightStickX * -0.75,
                    true
            );
        } else {
            // Direct drive control (old method)
            follower.setTeleOpDrive(
                    -gamepad1LeftStickY,
                    -gamepad1LeftStickX,
                    gamepad1RightStickX * -0.75,
                    true
            );
        }

        timeDrive = sectionTimer.milliseconds();

        // ===== APRILTAG LOCALIZATION =====
        sectionTimer.reset();

        // Update Camera Offset from MDOConstants (allows for live tuning)
        if (localizer != null) {
            localizer.setCameraPose(new Position(DistanceUnit.INCH,
                    MDOConstants.CameraOffset[0], MDOConstants.CameraOffset[1], MDOConstants.CameraOffset[2], 0),
                    localizer.cameraOrientation);
        }

        // AprilTag pose is cached in background thread at configurable frequency
        // Only update follower pose if we have a valid tag and good confidence
        if (aprilPose != null && MDOConstants.useAprilTags && localizer != null) {
            double decisionMargin = localizer.getDecisionMargin();
            if (decisionMargin > 0.8 && aprilSlowdownTimer.milliseconds() > 100) {
                follower.setPose(new Pose(aprilPose[0], aprilPose[1], aprilPose[3] + Math.toRadians(90.0)));
                aprilSlowdownTimer.reset();
            }
        }

        timeAprilTag = sectionTimer.milliseconds();

        // ===== SERVO CONTROL =====
        sectionTimer.reset();

        aimingLoop();

        timeServo = sectionTimer.milliseconds();

        // ===== LAUNCHER CONTROL =====
        sectionTimer.reset();

        if (gamepad2.dpad_down && gamepadTimerMs > 300) {
            gamepadTimer.reset();
            if (!launcherSpinning) {
                targetPower = MDOConstants.launchPower;
                launcherSpinning = true;
            } else {
                targetPower = 0;
                launcherSpinning = false;
            }
        }

        if (gamepad2.right_bumper && lifterAutoLaunchTimer.milliseconds() > 650) { // launch purple
            lifterAutoLaunchTimer.reset();
            // Use launchCached to avoid blocking the loop with sensor reads during launch
            sorterController.launchCached(RobotCoreCustom.CustomSorterController.CustomColor.PURPLE);
        }
        if (gamepad2.left_bumper && lifterAutoLaunchTimer.milliseconds() > 650) {
            lifterAutoLaunchTimer.reset();
            // Use launchCached to avoid blocking the loop with sensor reads during launch
            sorterController.launchCached(RobotCoreCustom.CustomSorterController.CustomColor.GREEN);
        }

        //launcherMotors.setPower(launcherSpinning ? MDOConstants.launchPower : 0);

        // Clamp target power
        targetPower = Math.max(-1.0, Math.min(1.0, targetPower));
		launcherMotors.setRPM(launcherSpinning ? MDOConstants.LauncherRPM : 0);
		launcherMotors.setPIDFController(MDOConstants.LauncherPIDF);

        timeLauncher = sectionTimer.milliseconds();

        // ===== INTAKE CONTROL =====
        sectionTimer.reset();

        // Auto-Stop Logic:
        // Automatically turns off the intake when the 3rd ball is collected to save power.
        // Logic: If intake is ON AND we have 3+ balls AND we didn't have 3 balls last loop.
        // This allows the driver to manually turn the intake back ON even if 3 balls are held.
        if (intakeRunningState == intakeState.IN && currentBallCount >= 3 && prevBallCount < 3) {
            intakeRunningState = intakeState.STOP;
        }
        prevBallCount = currentBallCount;

         if (intakeInputTimer.milliseconds() > 300) {
             if (gamepad2.a) {
                 intakeInputTimer.reset();
                 if (intakeRunningState == intakeState.IN) {
                     intakeRunningState = intakeState.STOP;
                 } else {
                     intakeRunningState = intakeState.IN;
                 }
             } else if (gamepad2.b) {
                 intakeInputTimer.reset();
                 if (intakeRunningState == intakeState.OUT) {
                     intakeRunningState = intakeState.STOP;
                 } else {
                     intakeRunningState = intakeState.OUT;
                 }
             }
         }

            switch (intakeRunningState) {
                case IN:
                    intakeMotor.setPower(0.8);
                    break;
                case OUT:
                    intakeMotor.setPower(-1);
                    break;
                case STOP:
                default:
                    intakeMotor.setPower(0);
                    break;
            }
        timeIntake = sectionTimer.milliseconds();

        // ===== TELEMETRY BLOCK - Throttled to reduce overhead =====
        sectionTimer.reset();

        telemetryLoopCounter++;
        if (telemetryLoopCounter >= TELEMETRY_UPDATE_INTERVAL) {
            telemetryLoopCounter = 0; // Reset counter
            telemetryC.addData("Team", team.toString());
            telemetryC.addData("External Heading (deg)", robotHeadingRad);
            telemetryC.addData("Pose X", poseX);
            telemetryC.addData("Pose Y", poseY);

            if (aprilPose != null) {
                telemetryC.addData("X (in) AprilTag", aprilPose[0]);
                telemetryC.addData("Y (in) AprilTag", aprilPose[1]);
            } else {
                telemetryC.addData("X (in) AprilTag", "No Tag");
                telemetryC.addData("Y (in) AprilTag", "No Tag");
            }

            if (launchVectors != null) {
                telemetryC.addData("Launch Elevation (deg)", launchElevationDeg);
                telemetryC.addData("Elevation Servo Pos", elevationServoFinal);
                telemetryC.addData("Launch Azimuth (deg)", launchAzimuthDeg);
                telemetryC.addData("Robot Field Azimuth (deg)", robotFieldRelativeAzimuthDeg);
                telemetryC.addData("Field Relative Azimuth (deg)", fieldRelativeAzimuthDeg);
                telemetryC.addData("Final Azimuth (deg)", finalAzimuthDeg);
                telemetryC.addData("Servo Pos (deg)", servoPosition);
                telemetryC.addData("Servo Target (deg)", servoTarget);
            } else {
                telemetryC.addData("Launch Vectors", "Target Unreachable");
            }

            telemetryC.addData("Launcher RPM", launcherRPM);
            telemetryC.addData("Launcher Pressure", targetPower); // Corrected label to Launcher Power, but kept logic
	        telemetryC.addData("launcherRunning", launcherSpinning);
            telemetryC.addData("Intake State", intakeRunningState);
            telemetryC.addData("Ball Count", currentBallCount);

            // Cached color sensor data (updated in background thread)
            telemetryC.addData("Color 0", sorterController.getCachedColor(0));
            telemetryC.addData("Color 1", sorterController.getCachedColor(1));
            telemetryC.addData("Color 2", sorterController.getCachedColor(2));


            // ===== CPU USAGE/TEMP TELEMETRY =====
            telemetryC.addData("CPU Usage (%)", cpuUsage);
            telemetryC.addData("CPU Temp (°C)", cpuTemp);


            // ===== PERFORMANCE TIMING TELEMETRY =====
            telemetryC.addData("Loop Time (ms)", loopTimeMs);

            double totalTrackedTime = timeCaching + timeUpdate + timeSorter + timeDrive +
                                      timeAprilTag + timeServo + timeLauncher + timeTelemetry;
            double unaccountedTime = loopTimeMs - totalTrackedTime;

            telemetryC.addData("--- Performance Breakdown ---", "");
            telemetryC.addData("Caching", String.format("%.2f ms", timeCaching));
            telemetryC.addData("Update/PIDF", String.format("%.2f ms", timeUpdate));
            telemetryC.addData("Sorter Updates", String.format("%.2f ms", timeSorter));
            telemetryC.addData("Drive Control", String.format("%.2f ms", timeDrive));
            telemetryC.addData("AprilTag", String.format("%.2f ms", timeAprilTag));
            telemetryC.addData("Servo Control", String.format("%.2f ms", timeServo));
            telemetryC.addData("Launcher Control", String.format("%.2f ms", timeLauncher));
            telemetryC.addData("Telemetry", String.format("%.2f ms", timeTelemetry));
            telemetryC.addData("--- Summary ---", "");
            telemetryC.addData("Total Tracked", String.format("%.2f ms", totalTrackedTime));
            telemetryC.addData("Unaccounted Time", String.format("%.2f ms (%.1f%%)",
                unaccountedTime, (unaccountedTime / loopTimeMs) * 100));
            telemetryC.addData("lifterTimer", lifterAutoLaunchTimer.milliseconds());

            // Update Telemetry
            telemetryC.update();
        }

        timeTelemetry = sectionTimer.milliseconds();
        loopTimeMs = loopTimer.milliseconds();
    }
    public void aimingLoop() {
        // Calculate elevation angle for launch
        // start off with IMU offset
        // Pre-calculate launch vector conversions if available

        if (launchVectors != null) {
            launchAzimuthDeg = Math.toDegrees(launchVectors[1]);
            // Elevation is already calculated as servo position in LocalizationAutoAim, not radians
            launchElevationDeg = launchVectors[0];
        }

        if (launchElevationDeg != null) {
            elevationServoTarget = (launchElevationDeg + MDOConstants.ElevationOffset) * MDOConstants.ElevationMultiplier;
            // Fix clamp logic to correctly bound between -0.4 and 1.0
            elevationServoFinal = Math.max(-0.4, Math.min(1.0, elevationServoTarget));
            elevationServo.setPosition(elevationServoFinal);
        }

        // Wrap-around for launchAzimuthDeg: Normalize to 0-360 degree range
        if (launchAzimuthDeg != null) {
            launchAzimuthDeg = launchAzimuthDeg % 360.0;
            if (launchAzimuthDeg < 0) {
                launchAzimuthDeg += 360.0;
            }
        }

        if (MDOConstants.EnableTurretIMUCorrection && MDOConstants.EnableTurret) {
            fieldRelativeAzimuthDeg = robotFieldRelativeAzimuthDeg + MDOConstants.AzimuthIMUOffset;

            // Wrap-around for fieldRelativeAzimuthDeg with offset
            fieldRelativeAzimuthDeg = fieldRelativeAzimuthDeg % 360.0;
            if (fieldRelativeAzimuthDeg < 0) {
                fieldRelativeAzimuthDeg += 360.0;
            }

            fieldRelativeAzimuthDeg = fieldRelativeAzimuthDeg * MDOConstants.AzimuthMultiplier;

            if (MDOConstants.EnableLauncherCalcAzimuth && launchAzimuthDeg != null) {
                finalAzimuthDeg = fieldRelativeAzimuthDeg + launchAzimuthDeg;
            } else {
                finalAzimuthDeg = fieldRelativeAzimuthDeg;
            }

            // Apply wrap-around offset before normalization, then remove it after
            // This shifts WHERE the wrap-around occurs without changing the main offset
            finalAzimuthDeg += MDOConstants.AzimuthWrapAroundOffset;

            // Wrap-around: Normalize final angle to 0-360 degree range
            finalAzimuthDeg = finalAzimuthDeg % 360.0;
            if (finalAzimuthDeg < 0) {
                finalAzimuthDeg += 360.0;
            }

            // Remove the wrap-around offset so it only affects where wrapping occurs
            finalAzimuthDeg -= MDOConstants.AzimuthWrapAroundOffset;

            // Map 0-360 degrees to -1 to 1 range for servo position
            double servoPosition = (finalAzimuthDeg / 180.0) - 1.0;

            // Update PID coefficients if changed via Panels (has built-in change detection)
            azimuthServo.setPIDCoefficients(MDOConstants.AzimuthPIDFConstants);

            // Set Azimuth Servos with mapped value (-1 to 1)
            if (MDOConstants.EnableTurret) {
                azimuthServo.setPosition(-servoPosition);
            } else {
                azimuthServo.setPosition(0.0);
            }// First ratio is 96:20
            // Second ratio is 25:120
            // Combined ratio is 96*25 : 20*120 = 2400 : 2400 = 1:1
        }
    }
}
