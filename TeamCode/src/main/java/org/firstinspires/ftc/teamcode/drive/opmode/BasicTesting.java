package org.firstinspires.ftc.teamcode.drive.opmode;


import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.drive.CustomPIDFController;
import org.firstinspires.ftc.teamcode.drive.RobotCoreCustom;

import java.text.DecimalFormat;

@TeleOp(name = "BasicTesting", group = "!advanced")
public class BasicTesting extends OpMode {
    //private AprilTagLocalizer localizer;
    IMU imuEX;
    RobotCoreCustom robotCoreCustom;
    Follower follower;
    DecimalFormat df = new DecimalFormat("#.##");
    //AprilTagLocalizer localizer;
    //Servo elevationServo;
    RobotCoreCustom.CustomMotor launcher0, launcher1;

    @Override
    public void init() {;
        robotCoreCustom = new RobotCoreCustom(hardwareMap);
        launcher0 = new RobotCoreCustom.CustomMotor( hardwareMap, "rr", true, 28, new CustomPIDFController(0.1, 0.01, 0.005, 0.0));
        launcher1 = new RobotCoreCustom.CustomMotor( hardwareMap, "rf", true, 28, new CustomPIDFController(0.1, 0.01, 0.005, 0.0));

    }

    @Override
    public void loop() {
        launcher0.setPower(gamepad1.right_stick_y);
        launcher1.setPower(-gamepad1.right_stick_y);
        double launcher0RPM = launcher0.getRPM();
        telemetry.addData("Launcher0 RPM", df.format(launcher0RPM));
        double launcher1RPM = launcher1.getRPM();
        telemetry.addData("Launcher1 RPM", df.format(launcher1RPM));
        telemetry.update();
    }
}