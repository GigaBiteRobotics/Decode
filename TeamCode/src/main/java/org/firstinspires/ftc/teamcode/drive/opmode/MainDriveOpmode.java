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
    Servo elevationServo, azimuthServo0, azimuthServo1;
    Double launchElevationDeg = 0.0;
    double elevationServoTarget = 0.0;
    RobotCoreCustom.CustomMotor launcher;
    Double[] lastAprilLocalization = null;
    double targetPower = 0.0;
    ElapsedTime gamepadTimer = new ElapsedTime();
    RobotCoreCustom.CustomTelemetry telemetryC;

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
        if (gamepad1.a) { team = Team.RED; }
        else { team = Team.BLUE; }
        if (team == Team.RED) {
            MDOConstants.targetLocation = new Double[]{-70.0, -70.0, 40.0};
        } else {
            MDOConstants.targetLocation = new Double[]{-70.0, 70.0, 40.0};
        }
        robotCoreCustom = new RobotCoreCustom(hardwareMap, follower);
        localizer = new AprilTagLocalizer();
        elevationServo = hardwareMap.get(Servo.class, "elevationServo");
        azimuthServo0 = hardwareMap.get(Servo.class, "azimuthServo0");
        azimuthServo1 = hardwareMap.get(Servo.class, "azimuthServo1");
        localizer.initAprilTag(hardwareMap, "Webcam 1");

        launcher = new RobotCoreCustom.CustomMotor(hardwareMap, "launcher0", true, 28, MDOConstants.launcherPIDF);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleopDrive();
        gamepadTimer.reset();
        aprilSlowdownTimer.reset();
        customThreads = new CustomThreads(robotCoreCustom, follower);
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


        // Launch calculations

        Double[] launchVectors = RobotCoreCustom.localizerLauncherCalc(follower, MDOConstants.targetLocation);

        // Launcher data
        double launcherRPM = launcher.getRPM();

        // Gamepad inputs - all sticks
        double gamepad1LeftStickY = gamepad1.left_stick_y;
        double gamepad1LeftStickX = gamepad1.left_stick_x;
        double gamepad1RightStickX = gamepad1.right_stick_x;
        double gamepad2RightStickY = gamepad2.right_stick_y;
        boolean gamepad2A = gamepad2.a;
        boolean gamepad2B = gamepad2.b;

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
        launcher.setPIDFController(pidfConstant);
        launcher.updateRPMPID();

        follower.update();
        timeUpdate = sectionTimer.milliseconds();

        // ===== DRIVE CONTROL =====
        sectionTimer.reset();

        follower.setTeleOpDrive(
                -gamepad1LeftStickY,
                -gamepad1LeftStickX,
                gamepad1RightStickX * -0.67,
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

        // ===== SERVO CONTROL =====
        sectionTimer.reset();

        // Pre-calculate launch vector conversions if available
        Double launchAzimuthDeg = null;
        double fieldRelativeAzimuthDeg = 0;
        double maxRotations = MDOConstants.maxTurretAzimuthRotations; // Maximum servo rotations (1.0 = 360°, 2.0 = 720°, etc.)

        if (launchVectors != null) {
            launchElevationDeg = Math.toDegrees(launchVectors[0]);
            launchAzimuthDeg = Math.toDegrees(launchVectors[1]);

            // Offset azimuth by robot heading
            double fieldRelativeAzimuth = launchVectors[1] - robotHeadingRad;

            // Wrap around: normalize to -PI to PI range
            fieldRelativeAzimuth = Math.atan2(Math.sin(fieldRelativeAzimuth), Math.cos(fieldRelativeAzimuth));

            // Store the current servo position to find shortest path
            double currentServoPos = azimuthServo0.getPosition();

            // Convert field relative azimuth to 0 to 2*PI range
            double targetAzimuth = fieldRelativeAzimuth;
            if (targetAzimuth < 0) {
                targetAzimuth += 2 * Math.PI;
            }

            // Map to servo range (0.0 to 1.0)
            double targetNormalized = targetAzimuth / (2 * Math.PI);

            // Find shortest path considering wrap-around
            double directDistance = Math.abs(targetNormalized - currentServoPos);
            double wrapDistance = 1.0 - directDistance;

            double finalServoPos;
            if (directDistance <= wrapDistance) {
                // Direct path is shorter
                finalServoPos = targetNormalized;
            } else {
                // Wrapped path is shorter
                if (targetNormalized > currentServoPos) {
                    // Wrap backwards (through 0)
                    finalServoPos = targetNormalized - 1.0;
                } else {
                    // Wrap forwards (through 1.0)
                    finalServoPos = targetNormalized + 1.0;
                }
            }

            // Clamp to valid servo range considering max rotations
            finalServoPos = Math.max(0.0, Math.min(maxRotations, finalServoPos));

            // Calculate servo positions
            elevationServoTarget = launchVectors[0] / Math.toRadians(45.0);

            elevationServo.setPosition(elevationServoTarget);
            azimuthServo0.setPosition(finalServoPos);
            fieldRelativeAzimuthDeg = Math.toDegrees(fieldRelativeAzimuth);
        }

        timeServo = sectionTimer.milliseconds();

        // ===== LAUNCHER CONTROL =====
        sectionTimer.reset();

        if (usePIDFLauncher) {
            int targetRPM = (int) (targetPower * 5500);
            launcher.setRPM(targetRPM);
        } else {
            launcher.setPower(targetPower);
        }

        // Gamepad controls for target power
        if (gamepad2A) {
            targetPower = 0.0;
        }
        if (gamepad2B) {
            targetPower = 0.82;
        }
        if (gamepadTimerMs > 200 && (gamepad2RightStickY > 0.1 || gamepad2RightStickY < -0.1)) {
            targetPower += gamepad2RightStickY * 0.1;
            gamepadTimer.reset();
        }

        // Clamp target power
        targetPower = Math.max(-1.0, Math.min(1.0, targetPower));

        timeLauncher = sectionTimer.milliseconds();

        // Cache loop time at the end for accurate measurement


        // ===== TELEMETRY BLOCK - Update every 3 loops to reduce overhead =====
        sectionTimer.reset();

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
            telemetryC.addData("Launch Elevation (deg)", elevationServoTarget);
            telemetryC.addData("Launch Azimuth (deg)", launchAzimuthDeg);
            telemetryC.addData("Launch Azimuth (deg, offset)", fieldRelativeAzimuthDeg);
        } else {
            telemetryC.addData("Launch Vectors", "Target Unreachable");
        }

        telemetryC.addData("Launcher RPM", launcherRPM);
        telemetryC.addData("Launcher Power", targetPower);

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
}