package org.firstinspires.ftc.teamcode.drive.opmode;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.drive.RobotCoreCustom;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.text.DecimalFormat;

@TeleOp(name = "BasicMechanum", group = "!advanced")
public class BasicMechanum extends OpMode {
    //private AprilTagLocalizer localizer;
    IMU imuEX;
    RobotCoreCustom robotCoreCustom;
    Follower follower;
    DecimalFormat df = new DecimalFormat("#.##");
    //AprilTagLocalizer localizer;
    //Servo elevationServo;
    DcMotor launcher0, launcher1;

    @Override
    public void init() {;
        robotCoreCustom = new RobotCoreCustom(hardwareMap);

        launcher0 = hardwareMap.get(DcMotor.class, "rr");
        launcher1 = hardwareMap.get(DcMotor.class, "rf");
    }

    @Override
    public void loop() {
        launcher0.setPower(gamepad1.right_stick_y);
        launcher1.setPower(-gamepad1.right_stick_y);
        double launcher0RPM = robotCoreCustom.getRPM(launcher0, 28);
        telemetry.addData("Launcher0 RPM", df.format(launcher0RPM));
        double launcher1RPM = robotCoreCustom.getRPM(launcher1, 28);
        telemetry.addData("Launcher1 RPM", df.format(launcher1RPM));
        telemetry.update();
    }
}