package org.firstinspires.ftc.teamcode.drive.opmode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    double targetPower = 0.0;
    // timer
    ElapsedTime gamepadTimer = new ElapsedTime();
    static TelemetryManager telemetryM;

	@Override
    public void init() {
        ;
        robotCoreCustom = new RobotCoreCustom(hardwareMap, follower);
        localizer = new AprilTagLocalizer();
        elevationServo = hardwareMap.get(Servo.class, "elevationServo");
        localizer.initAprilTag(hardwareMap, "Webcam 1");

        launcher = new RobotCoreCustom.CustomMotor(hardwareMap, "launcher0", true, 28, MDOConstants.launcherPIDF);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleopDrive();
        gamepadTimer.reset();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        launcher.setPidfController(MDOConstants.launcherPIDF);
        launcher.updateRPMPID();
        follower.update();
        robotCoreCustom.drawCurrentAndHistory(follower);
        /*
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x * -0.67, true // 67 hehe
        );

         */
        telemetry.addData("launcherPower: ", targetPower);
        if (localizer.isLocalized()) {
            Double[] aprilPose = localizer.getPosition();
            telemetry.addData("X (in) AprilTag", aprilPose[0].toString());
            telemetry.addData("Y (in) AprilTag", aprilPose[1].toString());
            telemetry.addData("Y (in) AprilTag", aprilPose[1].toString());
            if (MDOConstants.useAprilTags) {
                follower.setPose(new Pose(aprilPose[0], aprilPose[1], aprilPose[3] + Math.toRadians(90.0)));
            } else return;
        }
        Double[] launchVectors = RobotCoreCustom.localizerLauncherCalc(follower, MDOConstants.targetLocation);
        telemetry.addData("External Heading (rad)", RobotCoreCustom.getExternalHeading());
        telemetry.addData("Pose X: ", df.format(follower.getPose().getX()));
        telemetry.addData("Pose Y: ", df.format(follower.getPose().getY()));
        if (launchVectors != null) {
            telemetry.addData("Launch Elevation (deg)", df.format(Math.toDegrees(launchVectors[0])));
            telemetry.addData("Launch Azimuth (deg)", df.format(Math.toDegrees(launchVectors[1])));
        } else {
            telemetry.addData("Launch Vectors: ", "Target Unreachable");
        }

        // Get robot heading from IMU in radians
        double robotHeadingRad = follower.getHeading();

        if (launchVectors != null) {
            // Offset azimuth by robot heading
            double fieldRelativeAzimuth = launchVectors[1] - robotHeadingRad;

            // Clamp and normalize as before
            double minAngleRad = Math.toRadians(0);
            double maxAngleRad = Math.toRadians(60);
            double normalized = (fieldRelativeAzimuth - minAngleRad) / (maxAngleRad - minAngleRad);
            normalized = Math.max(0, Math.min(1, normalized));
            elevationServo.setPosition(normalized);

            telemetry.addData("Launch Azimuth (deg, offset): ", df.format(Math.toDegrees(fieldRelativeAzimuth)));
        }
        //launcher.setRPM((int) (gamepad1.right_stick_y * 4000));
        if (MDOConstants.usePIDFLauncher) {
            launcher.setRPM((int) (targetPower * 5500));
        } else { launcher.setPower(targetPower); }

        double launcherRPM = launcher.getRPM();
        telemetry.addData("Launcher RPM: ", df.format(launcherRPM));
        //telemetry.addData("Target Launcher RPM", df.format((int) (gamepad1.right_stick_y * 4000)));
        telemetryM.debug("External Heading (deg)", RobotCoreCustom.getExternalHeading());
        telemetryM.debug("Pose X: "+ df.format(follower.getPose().getX()));
        telemetryM.debug("Pose Y: "+ df.format(follower.getPose().getY()));
        telemetryM.update();

        if (gamepad1.a) { targetPower = 0.0; }
        if (gamepad1.b) { targetPower = 0.78; }
        if (gamepadTimer.milliseconds() > 200 & (gamepad1.right_stick_y > 0.1 || gamepad1.right_stick_y < -0.1)) {
            targetPower += gamepad1.right_stick_y * 0.1;
            gamepadTimer.reset();
        }
        if (targetPower > 1.0) targetPower = 1.0;
        if (targetPower < -1.0) targetPower = -1.0;
    }
    }
