package org.firstinspires.ftc.teamcode.drive.opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.drive.RobotCoreCustom;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Drive", group = "!advanced")
public class MainDriveOpmode extends OpMode {
    private AprilTagLocalizer localizer;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    Follower follower;
    IMU imuEX;
    RobotCoreCustom robotCoreCustom;

    @Override
    public void init() {
        robotCoreCustom = new RobotCoreCustom(hardwareMap);
        localizer = new AprilTagLocalizer(hardwareMap, telemetry);
        imuEX = hardwareMap.get(IMU.class, "imuEX");
        // Optional: Configure camera and offsets for your robot
        localizer.setCameraConfig("webcam", 1920, 1080);
        localizer.setCameraOffsets(6.0, 0.0, 2.0); // forward, side, height offsets
        localizer.setConfidenceThreshold(0.85);
        localizer.setTelemetryEnabled(true);
        follower = Constants.createFollower(hardwareMap);
        localizer.init();
    } //test

    @Override
    public void loop() {
        // Update the localizer to process the latest camera frame
        localizer.update();

        // Retrieve and display the robot's pose if available
        if (localizer.isLocalized()) {
            double[] pose = localizer.getRobotPose();
            telemetry.addData("X (in) AprilTag", pose[0]);
            telemetry.addData("Y (in) AprilTag", pose[1]);
            telemetry.addData("Heading (deg) AprilTag", Math.toDegrees(pose[2]));
            follower.setPose(new Pose(pose[0], pose[1], RobotCoreCustom.getExternalHeading()));
        } else {
            telemetry.addData("Pose", follower.getPose().toString());
        }
        telemetry.update();
    }
}