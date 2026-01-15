package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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
    RobotCoreCustom.CustomMotor launcher0, launcher1, drawbridgeMotor;

    // Aiming servos
    Servo elevationServo, azimuthServo0, azimuthServo1;

    // Aiming state variables
    double targetElevation = 0.0;      // Target elevation angle in radians
    double targetAzimuth = 0.0;        // Target azimuth angle in radians
    double currentElevation = 0.0;     // Current smoothed elevation
    double currentAzimuth = 0.0;       // Current smoothed azimuth
    double manualElevationOffset = 0.0; // Manual adjustment to elevation
    double manualAzimuthOffset = 0.0;   // Manual adjustment to azimuth
    boolean aimLocked = false;          // Whether aim is within acceptable error
    ElapsedTime manualAdjustTimer = new ElapsedTime();

    Double[] lastAprilLocalization = null;
    double targetPower = 0.0;
    ElapsedTime gamepadTimer = new ElapsedTime();
    RobotCoreCustom.CustomTelemetry telemetryC;
    RobotCoreCustom.CustomSorterController sorterController;
    boolean launcherSpinning = false;
    int targetRPM = 0;

    enum Team {
        RED,
        BLUE
    }

    Team team = Team.BLUE;
     ElapsedTime aprilSlowdownTimer = new ElapsedTime();
     ElapsedTime loopTimer = new ElapsedTime();
    double loopTimeMs = 0;
    ElapsedTime sectionTimer = new ElapsedTime();
    double timeCaching = 0, timeUpdate = 0, timeDrive = 0;
    double timeAprilTag = 0, timeServo = 0, timeLauncher = 0, timeTelemetry = 0;

    static TelemetryManager telemetryM;

    // Threads
    CustomThreads customThreads;

	@Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryC = new RobotCoreCustom.CustomTelemetry(telemetry, telemetryM);

        // Sorting Initialization
        sorterController = new RobotCoreCustom.CustomSorterController(hardwareMap);

        // Get data from autonomous if available
        AutoToTeleDataTransferer dataTransfer = AutoToTeleDataTransferer.getInstance();
        Pose startPose = dataTransfer.getEndPose();
        String allianceColor = dataTransfer.getAllianceColor();

        // Set team based on auto data or gamepad input
        if (!allianceColor.equals("UNKNOWN")) {
            // Use alliance color from auto
            team = allianceColor.equals("RED") ? Team.RED : Team.BLUE;
            telemetryC.addData("Alliance from Auto", allianceColor);
        } else {
            // Manual selection if no auto data
            if (gamepad1.a) { team = Team.RED; }
            else { team = Team.BLUE; }
        }

        if (team == Team.RED) {
            MDOConstants.targetLocation = new Double[]{-70.0, -70.0, 40.0};
        } else {
            MDOConstants.targetLocation = new Double[]{-70.0, 70.0, 40.0};
        }

        robotCoreCustom = new RobotCoreCustom(hardwareMap, follower);
        localizer = new AprilTagLocalizer();
        localizer.initAprilTag(hardwareMap, "Webcam 1");

        // Initialize aiming servos
        try {
            elevationServo = hardwareMap.get(Servo.class, "elevationServo");
            azimuthServo0 = hardwareMap.get(Servo.class, "azimuthServo0");
            if (MDOConstants.useDualAzimuthServos) {
                azimuthServo1 = hardwareMap.get(Servo.class, "azimuthServo1");
            }
            telemetryC.addData("Aiming Servos", "Initialized");
        } catch (Exception e) {
            telemetryC.addData("Servo Init Error", e.getMessage());
        }

        launcher0 = new RobotCoreCustom.CustomMotor(hardwareMap, "launcher0", true, 28, MDOConstants.launcherPIDF);
        launcher1 = new RobotCoreCustom.CustomMotor(hardwareMap, "launcher1", false, 28, MDOConstants.launcherPIDF);
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

        follower.startTeleopDrive();
        gamepadTimer.reset();
        aprilSlowdownTimer.reset();
        customThreads = new CustomThreads(robotCoreCustom, follower);

        // Display auto summary if available
        if (dataTransfer.isAutoCompleted()) {
            telemetryC.addData("Auto Status", "Completed Successfully");
            telemetryC.addData("Auto Runtime", String.format("%.2fs", dataTransfer.getAutoRuntime()));
        } else if (!allianceColor.equals("UNKNOWN")) {
            telemetryC.addData("Auto Status", "Did not complete");
        }

        telemetryC.update();
    }

    @Override
    public void start() {
        customThreads.startDrawingThread();
        customThreads.startCPUMonThread();
    }
    @Override
    public void stop() {
        customThreads.stopDrawingThread();
        customThreads.stopCPUMonThread();
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


        // Launcher data
        double launcherRPM = launcher0.getRPM();

        // Gamepad inputs - all sticks
        double gamepad1LeftStickY = gamepad1.left_stick_y;
        double gamepad1LeftStickX = gamepad1.left_stick_x;
        double gamepad1RightStickX = gamepad1.right_stick_x;

        // Timer values
        double gamepadTimerMs = gamepadTimer.milliseconds();

        // Constants used multiple times
        CustomPIDFController pidfConstant = MDOConstants.launcherPIDF;
        boolean usePIDFLauncher = MDOConstants.usePIDFLauncher;

        // Cache AprilTag position ONCE to avoid multiple blocking calls
        Double[] aprilPose = localizer.getPosition();
        timeCaching = sectionTimer.milliseconds();

        // ===== MAIN LOOP LOGIC =====
        sectionTimer.reset();
        launcher0.setPIDFController(pidfConstant);
        launcher0.updateRPMPID();

        follower.update();
        timeUpdate = sectionTimer.milliseconds();
        sorterController.lifterUpdater();
        if (gamepad2.right_bumper) {
            sorterController.launcherViaIndex(1);
        }
        //sorterController.lightingUpdater();

        // ===== DRIVE CONTROL =====
        sectionTimer.reset();

        follower.setTeleOpDrive(
                -gamepad1LeftStickY,
                -gamepad1LeftStickX,
                gamepad1RightStickX * -0.75,
                true
        );

        timeDrive = sectionTimer.milliseconds();

        // ===== APRILTAG LOCALIZATION =====
        sectionTimer.reset();

        if (aprilPose != null) {
            if (MDOConstants.useAprilTags &&
                    aprilSlowdownTimer.milliseconds() > 100 &&
                    localizer.getDecisionMargin() > 0.8) {

                follower.setPose(new Pose(aprilPose[0], aprilPose[1], aprilPose[3] + Math.toRadians(90.0)));
                aprilSlowdownTimer.reset();
            }
        }

        timeAprilTag = sectionTimer.milliseconds();

        // ===== SERVO CONTROL (AIMING SYSTEM) =====
        sectionTimer.reset();

        aimingLoop(robotHeadingRad, gamepadTimerMs);

        timeServo = sectionTimer.milliseconds();

        // ===== LAUNCHER CONTROL =====
        sectionTimer.reset();

        if (gamepad2.x && gamepadTimerMs > 300) {
            gamepadTimer.reset();
            if (!launcherSpinning) {
                if (usePIDFLauncher) {
                    targetRPM = 5000;
                } else {
                    targetPower = 1;
                }
                launcherSpinning = true;
            } else {
                targetPower = 0;
                targetRPM = 0;
                launcherSpinning = false;
            }
        }

        if (usePIDFLauncher) {
            launcher0.setRPM(targetRPM);
            launcher1.setRPM(targetRPM);
        } else {
            launcher0.setPower(targetPower);
            launcher1.setPower(-targetPower);
        }

        // Clamp target power
        targetPower = Math.max(-1.0, Math.min(1.0, targetPower));

        timeLauncher = sectionTimer.milliseconds();


        // ===== TELEMETRY BLOCK - Update every 3 loops to reduce overhead =====
        sectionTimer.reset();

        telemetryC.addData("External Heading (rad)", robotHeadingRad);

        telemetryC.addData("Pose X", poseX);
        telemetryC.addData("Pose Y", poseY);

        if (aprilPose != null) {
            telemetryC.addData("X (in) AprilTag", aprilPose[0]);
            telemetryC.addData("Y (in) AprilTag", aprilPose[1]);
        } else {
            telemetryC.addData("X (in) AprilTag", "No Tag");
            telemetryC.addData("Y (in) AprilTag", "No Tag");
        }


        telemetryC.addData("Launcher RPM", launcherRPM);
        telemetryC.addData("Launcher Power", targetPower);

        // ===== AIMING TELEMETRY =====
        if (MDOConstants.enableAutoAiming) {
            telemetryC.addData("--- Aiming System ---", "");
            telemetryC.addData("Target Elevation (deg)", Math.toDegrees(targetElevation));
            telemetryC.addData("Current Elevation (deg)", Math.toDegrees(currentElevation));
            telemetryC.addData("Target Azimuth (deg)", Math.toDegrees(targetAzimuth));
            telemetryC.addData("Current Azimuth (deg)", Math.toDegrees(currentAzimuth));
            telemetryC.addData("Manual Elev Offset (deg)", Math.toDegrees(manualElevationOffset));
            telemetryC.addData("Manual Azim Offset (deg)", Math.toDegrees(manualAzimuthOffset));
            telemetryC.addData("Aim Locked", aimLocked ? "YES" : "NO");
        }
        // ===== CPU USAGE/TEMP TELEMETRY =====
        telemetryC.addData("CPU Usage (%)", String.format("%.2f %%", customThreads.getCpuUsage()));
        telemetryC.addData("CPU Temp (°C)", String.format("%.2f °C", customThreads.getCpuTemp()));


        // ===== PERFORMANCE TIMING TELEMETRY =====
        telemetryC.addData("Loop Time (ms)", loopTimeMs);
        telemetryC.addData("--- Performance Breakdown ---", "");
        telemetryC.addData("Caching", String.format("%.2f ms", timeCaching));
        telemetryC.addData("Update/PIDF", String.format("%.2f ms", timeUpdate));
        telemetryC.addData("Drive Control", String.format("%.2f ms", timeDrive));
        telemetryC.addData("AprilTag", String.format("%.2f ms", timeAprilTag));
        telemetryC.addData("Servo Control", String.format("%.2f ms", timeServo));
        telemetryC.addData("Launcher Control", String.format("%.2f ms", timeLauncher));
        telemetryC.addData("Telemetry", String.format("%.2f ms", timeTelemetry));

        // Update Telemetry
        telemetryC.update();

        timeTelemetry = sectionTimer.milliseconds();
        loopTimeMs = loopTimer.milliseconds();
    }

    /**
     * Aiming Loop - Calculates and applies azimuth/elevation servo positions
     * @param robotHeadingRad Current robot heading in radians
     * @param gamepadTimerMs Current gamepad timer value in milliseconds
     */
    public void aimingLoop(double robotHeadingRad, double gamepadTimerMs) {
        if (!MDOConstants.enableAutoAiming) {
            return; // Exit if aiming is disabled
        }

        // ===== STEP 1: Calculate Target Angles =====
        // Use launcher calculation to get required elevation and azimuth
        Double[] launchVectors = RobotCoreCustom.localizerLauncherCalc(follower, MDOConstants.targetLocation);

        if (launchVectors != null && launchVectors.length >= 2) {
            // launchVectors[0] = elevation (radians), launchVectors[1] = azimuth (radians)
            targetElevation = launchVectors[0];
            targetAzimuth = launchVectors[1];

            // Convert to field-relative azimuth if enabled
            if (MDOConstants.useFieldRelativeAzimuth) {
                // Adjust azimuth based on robot heading so target stays fixed on field
                targetAzimuth = targetAzimuth - robotHeadingRad;
                // Normalize to -PI to PI range
                targetAzimuth = Math.atan2(Math.sin(targetAzimuth), Math.cos(targetAzimuth));
            }
        }

        // ===== STEP 2: Manual Adjustments =====
        // Allow driver to fine-tune aiming with gamepad2 D-pad

        // D-pad up/down for elevation
        if (gamepad2.dpad_up && manualAdjustTimer.milliseconds() > MDOConstants.manualAdjustmentHoldTime) {
            manualElevationOffset += MDOConstants.manualElevationStep;
            manualAdjustTimer.reset();
        }
        if (gamepad2.dpad_down && manualAdjustTimer.milliseconds() > MDOConstants.manualAdjustmentHoldTime) {
            manualElevationOffset -= MDOConstants.manualElevationStep;
            manualAdjustTimer.reset();
        }

        // D-pad left/right for azimuth
        if (gamepad2.dpad_right && manualAdjustTimer.milliseconds() > MDOConstants.manualAdjustmentHoldTime) {
            manualAzimuthOffset += MDOConstants.manualAzimuthStep;
            manualAdjustTimer.reset();
        }
        if (gamepad2.dpad_left && manualAdjustTimer.milliseconds() > MDOConstants.manualAdjustmentHoldTime) {
            manualAzimuthOffset -= MDOConstants.manualAzimuthStep;
            manualAdjustTimer.reset();
        }

        // Reset manual offsets with gamepad2.y
        if (gamepad2.y && gamepadTimerMs > 300) {
            manualElevationOffset = 0.0;
            manualAzimuthOffset = 0.0;
            gamepadTimer.reset();
        }

        // ===== STEP 3: Apply Offsets and Clamp to Limits =====
        double finalTargetElevation = targetElevation + manualElevationOffset;
        double finalTargetAzimuth = targetAzimuth + manualAzimuthOffset;

        // Clamp to configured min/max ranges
        finalTargetElevation = Math.max(MDOConstants.elevationAngleMin,
                                       Math.min(MDOConstants.elevationAngleMax, finalTargetElevation));
        finalTargetAzimuth = Math.max(MDOConstants.azimuthAngleMin,
                                     Math.min(MDOConstants.azimuthAngleMax, finalTargetAzimuth));

        // ===== STEP 4: Apply Smoothing =====
        // Calculate error between target and current position
        double elevationDelta = finalTargetElevation - currentElevation;
        double azimuthDelta = finalTargetAzimuth - currentAzimuth;

        // Apply deadzone to prevent micro-adjustments and jitter
        if (Math.abs(elevationDelta) > MDOConstants.elevationDeadzone) {
            currentElevation += elevationDelta * MDOConstants.aimingSmoothingFactor;
        }
        if (Math.abs(azimuthDelta) > MDOConstants.azimuthDeadzone) {
            currentAzimuth += azimuthDelta * MDOConstants.aimingSmoothingFactor;
        }

        // ===== STEP 5: Check Aim Lock Status =====
        // Aim is "locked" when within acceptable error margins
        aimLocked = (Math.abs(elevationDelta) < MDOConstants.maxElevationError) &&
                   (Math.abs(azimuthDelta) < MDOConstants.maxAzimuthError);

        // ===== STEP 6: Convert Angles to Servo Positions =====

        // Elevation servo: map angle range to servo range (0.0 to 1.0)
        double elevationServoPos = (currentElevation - MDOConstants.elevationAngleMin) /
                                  (MDOConstants.elevationAngleMax - MDOConstants.elevationAngleMin);
        elevationServoPos = elevationServoPos * (MDOConstants.elevationServoMax - MDOConstants.elevationServoMin)
                          + MDOConstants.elevationServoMin;
        elevationServoPos += MDOConstants.elevationServoOffset;

        // Reverse if servo is mounted backwards
        if (MDOConstants.elevationServoReversed) {
            elevationServoPos = 1.0 - elevationServoPos;
        }

        // Azimuth servo: map angle range to servo range (0.0 to 1.0)
        double azimuthServoPos = (currentAzimuth - MDOConstants.azimuthAngleMin) /
                                (MDOConstants.azimuthAngleMax - MDOConstants.azimuthAngleMin);
        azimuthServoPos = azimuthServoPos * (MDOConstants.azimuthServoMax - MDOConstants.azimuthServoMin)
                        + MDOConstants.azimuthServoMin;
        azimuthServoPos += MDOConstants.azimuthServoOffset;

        // Reverse if servo is mounted backwards
        if (MDOConstants.azimuthServoReversed) {
            azimuthServoPos = 1.0 - azimuthServoPos;
        }

        // ===== STEP 7: Safety Clamp =====
        // Ensure servo positions stay within valid range
        elevationServoPos = Math.max(0.0, Math.min(1.0, elevationServoPos));
        azimuthServoPos = Math.max(0.0, Math.min(1.0, azimuthServoPos));

        // ===== STEP 8: Set Servo Positions =====
        try {
            // Set elevation servo
            if (elevationServo != null) {
                elevationServo.setPosition(elevationServoPos);
            }

            // Set primary azimuth servo
            if (azimuthServo0 != null) {
                azimuthServo0.setPosition(azimuthServoPos);
            }

            // Set secondary azimuth servo if enabled
            if (MDOConstants.useDualAzimuthServos && azimuthServo1 != null) {
                double servo1Pos = MDOConstants.azimuthServo1Mirrored ? (1.0 - azimuthServoPos) : azimuthServoPos;
                azimuthServo1.setPosition(servo1Pos);
            }
        } catch (Exception e) {
            // Silently handle servo errors to avoid spamming telemetry
        }
    }
}
