package org.firstinspires.ftc.teamcode.drive.opmode;


import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {;
        robotCoreCustom = new RobotCoreCustom(hardwareMap, follower);
    }

    @Override
    public void loop() {
        loopTimer.reset();
        telemetry.addData("loop ms", df.format(loopTimer.milliseconds()));

    }
}