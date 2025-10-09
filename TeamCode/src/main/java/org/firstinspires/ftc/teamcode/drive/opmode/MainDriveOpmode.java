package org.firstinspires.ftc.teamcode.drive.opmode;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.drive.RobotCoreCustom;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.text.DecimalFormat;

@TeleOp(name = "Drive", group = "!advanced")
public class MainDriveOpmode extends OpMode {
    //private AprilTagLocalizer localizer;
    IMU imuEX;
    RobotCoreCustom robotCoreCustom;
    Follower follower;
    DecimalFormat df = new DecimalFormat("#.##");
    AprilTagLocalizer localizer;
    Servo elevationServo;
    RobotCoreCustom.CustomMotor launcher;
    Double[] lastAprilLocalization = null;

    @Override
    public void init() {;
        robotCoreCustom = new RobotCoreCustom(hardwareMap);
        localizer = new AprilTagLocalizer(hardwareMap, telemetry);
        elevationServo = hardwareMap.get(Servo.class, "elevationServo");
        localizer.setCameraConfig("webcam", 1920, 1080);
        localizer.setCameraOffsets(6.0, 0.0, 2.0); // forward, side, height offsets
        localizer.setConfidenceThreshold(0.85);
        localizer.setTelemetryEnabled(true);

        launcher = new RobotCoreCustom.CustomMotor(hardwareMap, "launcher", true, 28);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        localizer.init();
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Update the localizer to process the latest camera frame
        //localizer.update();
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x * -0.63, true
        );

        if (localizer.isLocalized()) {
            Double[] aprilPose = localizer.getRobotPose();
            telemetry.addData("X (in) AprilTag", aprilPose[0]);
            telemetry.addData("Y (in) AprilTag", aprilPose[1]);
            telemetry.addData("Heading (deg) AprilTag", Math.toDegrees(aprilPose[2]));
            follower.setPose(new Pose(aprilPose[0], aprilPose[1], RobotCoreCustom.getExternalHeading()));
        }
        lastAprilLocalization = (localizer.isLocalized()) ? localizer.getRobotPose() : lastAprilLocalization;
        if (!localizer.isLocalized()) {
            if (lastAprilLocalization != null) {
                telemetry.addData("X (in) AprilTag", lastAprilLocalization[0]);
                telemetry.addData("Y (in) AprilTag", lastAprilLocalization[1]);
                telemetry.addData("Heading (deg) AprilTag", Math.toDegrees(lastAprilLocalization[2]));
            } else {
                telemetry.addData("X (in) AprilTag", "Unknown");
                telemetry.addData("Y (in) AprilTag", "Unknown");
                telemetry.addData("Heading (deg) AprilTag", "Unknown");
            }
        }

        Double[] launchVectors = RobotCoreCustom.localizerLauncherCalc(follower, new Double[]{-70.0, 70.0, 40.0});
        telemetry.addData("External Heading (deg)", RobotCoreCustom.getExternalHeading());
        telemetry.addData("Pose X: ", df.format(follower.getPose().getX()));
        telemetry.addData("Pose Y: ", df.format(follower.getPose().getY()));
        if (launchVectors != null) {
            telemetry.addData("Launch Elevation (deg)", df.format(Math.toDegrees(launchVectors[0])));
            telemetry.addData("Launch Azimuth (deg)", df.format(Math.toDegrees(launchVectors[1])));
        } else {
            telemetry.addData("Launch Vectors", "Target Unreachable");
        }

        // Get robot heading from IMU in radians
        double robotHeadingRad = follower.getHeading();

        if (launchVectors != null) {
            // Offset azimuth by robot heading
            double fieldRelativeAzimuth = launchVectors[1] + robotHeadingRad;

            // Clamp and normalize as before
            double minAngleRad = Math.toRadians(0);
            double maxAngleRad = Math.toRadians(60);
            double normalized = (fieldRelativeAzimuth - minAngleRad) / (maxAngleRad - minAngleRad);
            normalized = Math.max(0, Math.min(1, normalized));
            elevationServo.setPosition(normalized);

            telemetry.addData("Launch Azimuth (deg, offset)", df.format(Math.toDegrees(fieldRelativeAzimuth)));
        }

        double launcherRPM = launcher.getRPM();
        telemetry.addData("Launcher RPM", df.format(launcherRPM));
        telemetry.update();
    }
}