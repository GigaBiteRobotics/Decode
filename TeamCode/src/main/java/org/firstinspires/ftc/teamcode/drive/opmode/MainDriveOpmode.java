package org.firstinspires.ftc.teamcode.drive.opmode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.drive.CustomPIDFController;
import org.firstinspires.ftc.teamcode.drive.MDOConstants;
import org.firstinspires.ftc.teamcode.drive.RobotCoreCustom;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.text.DecimalFormat;

@TeleOp(name = "Drive", group = "!advanced")
public class MainDriveOpmode extends OpMode {
    AprilTagLocalizer localizer;
    //IMU imuEX;
    RobotCoreCustom robotCoreCustom;
    Follower follower;
    DecimalFormat df = new DecimalFormat("#.##");
    Servo elevationServo, azimuthServo0, azimuthServo1;
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
    static TelemetryManager telemetryM;

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

    }

    private int loopCounter = 0;

    @Override
    public void loop() {
        loopTimer.reset();

        // ===== CACHE ALL VALUES AT THE BEGINNING =====

        // Servo positions
        double elevationServoPos = elevationServo.getPosition();

        // Follower/Pose data
        Pose currentPose = follower.getPose();
        double poseX = currentPose.getX();
        double poseY = currentPose.getY();
        double robotHeadingRad = follower.getHeading();
        double externalHeading = RobotCoreCustom.getExternalHeading();

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

        // ===== MAIN LOOP LOGIC =====

        launcher.setPidfController(pidfConstant);
        launcher.updateRPMPID();
        follower.update();

        // Only draw every 3 loops to reduce overhead
        if (loopCounter % 3 == 0) {
            robotCoreCustom.drawCurrentAndHistory(follower);
        }

        follower.setTeleOpDrive(
                -gamepad1LeftStickY,
                -gamepad1LeftStickX,
                gamepad1RightStickX * -0.67,
                true
        );

        // AprilTag localization - now using cached value
        if (aprilPose != null) {
            lastAprilLocalization = aprilPose;

            if (MDOConstants.useAprilTags &&
                    aprilSlowdownTimer.milliseconds() > 100 &&
                    aprilPose != lastAprilLocalization &&
                    localizer.getDecisionMargin() > 0.9) {

                follower.setPose(new Pose(aprilPose[0], aprilPose[1], aprilPose[3] + Math.toRadians(90.0)));
                aprilSlowdownTimer.reset();
            }
        }

        // Pre-calculate launch vector conversions if available
        Double launchElevationDeg = null;
        Double launchAzimuthDeg = null;
        double fieldRelativeAzimuthDeg = 0;

        // Servo control logic
        if (launchVectors != null) {
            launchElevationDeg = Math.toDegrees(launchVectors[0]);
            launchAzimuthDeg = Math.toDegrees(launchVectors[1]);

            // Offset azimuth by robot heading
            double fieldRelativeAzimuth = launchVectors[1] - robotHeadingRad;

            // Clamp and normalize
            double minAngleRad = 0;
            double maxAngleRad = Math.toRadians(180);
            double normalized = (fieldRelativeAzimuth - minAngleRad) / (maxAngleRad - minAngleRad);
            normalized = Math.max(0.0, Math.min(1.0, normalized));

            // Calculate servo positions
            double elevationServoTarget = launchVectors[0] / Math.toRadians(45.0);

            elevationServo.setPosition(elevationServoTarget);
            azimuthServo0.setPosition(normalized);

            fieldRelativeAzimuthDeg = Math.toDegrees(fieldRelativeAzimuth);
        }

        // Launcher control
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

        // Cache loop time at the end for accurate measurement
        double loopTimeMs = loopTimer.milliseconds();

        // ===== TELEMETRY BLOCK - Update every 3 loops to reduce overhead =====
        if (loopCounter % 3 == 0) {
            telemetryC.addData("Launcher Elevation Servo Pos", elevationServoPos);
            telemetryC.addData("External Heading (deg)", externalHeading);
            telemetryC.addData("Pose X", poseX);
            telemetryC.addData("Pose Y", poseY);

            if (aprilPose != null) {
                telemetryC.addData("X (in) AprilTag", aprilPose[0]);
                telemetryC.addData("Y (in) AprilTag", aprilPose[1]);
            }

            if (launchVectors != null) {
                telemetryC.addData("Launch Elevation (deg)", launchElevationDeg);
                telemetryC.addData("Launch Azimuth (deg)", launchAzimuthDeg);
                telemetryC.addData("Launch Azimuth (deg, offset)", fieldRelativeAzimuthDeg);
            } else {
                telemetryC.addData("Launch Vectors", "Target Unreachable");
            }

            telemetryC.addData("Launcher RPM", launcherRPM);
            telemetryC.addData("Launcher Power", targetPower);
            telemetryC.addData("Loop Time (ms)", loopTimeMs);

            telemetryC.update();
        }

        loopCounter++;
    }
}
