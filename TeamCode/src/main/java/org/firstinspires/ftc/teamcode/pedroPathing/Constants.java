package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
		    .forwardZeroPowerAcceleration(-32.75)
		    .lateralZeroPowerAcceleration(-60.42)
		    .translationalPIDFCoefficients(new PIDFCoefficients(0.25, 0, 0.03, 0))
		    .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.1, 0))
		    .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.045, 0, 0.0018, 0.8, 0.02))
		    .useSecondaryTranslationalPIDF(false)
		    .useSecondaryDrivePIDF(false)
		    .useSecondaryHeadingPIDF(false)
            .mass(13.61);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
		    .xVelocity(59.95)
		    .yVelocity(45.12)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("rr")
            .forwardPodY(-5.295)
            .forwardTicksToInches(-0.00195915)
            .strafeEncoder_HardwareMapName("lf")
            .strafePodX(-5.37)
            .strafeTicksToInches(0.001975258)
            .IMU_HardwareMapName("imuEX")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                    )
            );

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                .build();
    }
}