package org.firstinspires.ftc.teamcode.drive.rippedCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "MainOpMode (Blocks to Java)")
public class MainDriveRipped extends LinearOpMode {

	private CRServo sr;
	private Servo lift1;
	private Servo lift0;
	private CRServo sl;
	private DcMotor bl;
	private DcMotor fr;
	private DcMotor fl;
	private DcMotor br;
	private DcMotor launcher2;
	private DcMotor intake;
	private DcMotor intakestage2;
	private DcMotor launcher;

	int Intake2Booleanrunning;
	ElapsedTime gamePadTime;
	int launcherSpeedInt;
	int intakebooleanrunning;
	int isLaunching;
	int isGoingBackward;

	/**
	 * Describe this function...
	 */
	private void checkGamepad() {
		if (0.2 < gamepad2.right_trigger) {
			Intake2Booleanrunning = 1;
		} else {
			Intake2Booleanrunning = 0;
		}
		if (gamepad2.a && gamePadTime.milliseconds() > 200) {
			gamePadTime.reset();
			if (intakebooleanrunning == 1) {
				intakebooleanrunning = 0;
			} else {
				intakebooleanrunning = 1;
			}
		}
		if (gamepad2.dpad_up && gamePadTime.milliseconds() > 200) {
			gamePadTime.reset();
			launcherSpeedInt = 4;
		}
		if (gamepad2.dpad_right && gamePadTime.milliseconds() > 200) {
			gamePadTime.reset();
			launcherSpeedInt = 3;
		}
		if (gamepad2.dpad_down && gamePadTime.milliseconds() > 200) {
			gamePadTime.reset();
			launcherSpeedInt = 2;
		}
		if (gamepad2.dpad_left && gamePadTime.milliseconds() > 200) {
			gamePadTime.reset();
			launcherSpeedInt = 1;
		}
		if (gamepad2.x && gamePadTime.milliseconds() > 200) {
			gamePadTime.reset();
			launcherSpeedInt = 0;
		}
		if (gamepad2.b) {
			launcherSpeedInt = -1;
			intakebooleanrunning = -1;
			Intake2Booleanrunning = -1;
		} else {
			if (intakebooleanrunning != 0 || Intake2Booleanrunning != 0 || launcherSpeedInt == 1 || launcherSpeedInt == 2 || launcherSpeedInt == 3 || launcherSpeedInt == 4) {
			} else {
				launcherSpeedInt = 0;
				intakebooleanrunning = 0;
				Intake2Booleanrunning = 0;
			}
			if (launcherSpeedInt == -1) {
				launcherSpeedInt = 0;
				intakebooleanrunning = 0;
				Intake2Booleanrunning = 0;
			}
		}
		if (gamepad2.right_bumper) {
			isGoingBackward = 1;
		} else {
			isGoingBackward = 0;
		}
		if (0.2 < gamepad2.right_trigger) {
			isLaunching = 1;
		} else {
			isLaunching = 0;
		}
		if (gamepad1.dpad_down || gamepad1.dpad_up) {
			if (gamepad1.dpad_up) {
				lift1.setPosition(1);
				lift0.setPosition(0);
			} else if (gamepad1.dpad_down) {
				lift1.setPosition(0);
				lift0.setPosition(1);
			}
		} else {
			lift0.setPosition(0.5);
			lift1.setPosition(0.5);
		}
	}

	/**
	 * This OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
	 * This code will work with either a Mecanum-Drive or an X-Drive train.
	 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
	 *
	 * Also note that it is critical to set the correct rotation direction for each motor. See details below.
	 *
	 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
	 * Each motion axis is controlled by one Joystick axis.
	 *
	 * 1) Axial -- Driving forward and backward -- Left-joystick Forward/Backward
	 * 2) Lateral -- Strafing right and left -- Left-joystick Right and Left
	 * 3) Yaw -- Rotating Clockwise and counter clockwise -- Right-joystick Right and Left
	 *
	 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
	 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
	 * the direction of all 4 motors (see code below).
	 */
	@Override
	public void runOpMode() {
		ElapsedTime runtime;
		ElapsedTime servoUpdateTimer;
		float axial;
		float lateral;
		float yaw;
		float frontLeftPower;
		float frontRightPower;
		float backLeftPower;
		float backRightPower;
		double max;

		sr = hardwareMap.get(CRServo.class, "sr");
		lift1 = hardwareMap.get(Servo.class, "lift1");
		lift0 = hardwareMap.get(Servo.class, "lift0");
		sl = hardwareMap.get(CRServo.class, "sl");
		bl = hardwareMap.get(DcMotor.class, "bl");
		fr = hardwareMap.get(DcMotor.class, "fr");
		fl = hardwareMap.get(DcMotor.class, "fl");
		br = hardwareMap.get(DcMotor.class, "br");
		launcher2 = hardwareMap.get(DcMotor.class, "launcher2");
		intake = hardwareMap.get(DcMotor.class, "intake");
		intakestage2 = hardwareMap.get(DcMotor.class, "intake stage2");
		launcher = hardwareMap.get(DcMotor.class, "launcher");

		runtime = new ElapsedTime();
		gamePadTime = new ElapsedTime();
		servoUpdateTimer = new ElapsedTime();
		gamePadTime.reset();
		// ########################################################################################
		// !!! IMPORTANT Drive Information. Test your motor directions. !!!!!
		// ########################################################################################
		//
		// Most robots need the motors on one side to be reversed to drive forward.
		// The motor reversals shown here are for a "direct drive" robot
		// (the wheels turn the same direction as the motor shaft).
		//
		// If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
		// that your motors are turning in the correct direction. So, start out with the reversals here, BUT
		// when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
		//
		// Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward.
		// Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
		// <--- Click blue icon to see important note re. testing motor directions.
		lift0.setDirection(Servo.Direction.REVERSE);
		lift1.setDirection(Servo.Direction.REVERSE);
		sl.setDirection(CRServo.Direction.FORWARD);
		sr.setDirection(CRServo.Direction.REVERSE);
		bl.setDirection(DcMotor.Direction.FORWARD);
		fr.setDirection(DcMotor.Direction.FORWARD);
		fl.setDirection(DcMotor.Direction.REVERSE);
		br.setDirection(DcMotor.Direction.FORWARD);
		launcher2.setDirection(DcMotor.Direction.FORWARD);
		intake.setDirection(DcMotor.Direction.FORWARD);
		intakestage2.setDirection(DcMotor.Direction.FORWARD);
		launcher.setDirection(DcMotor.Direction.REVERSE);
		launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		// Wait for the game to start (driver presses START)
		telemetry.addData("Status", "Initialized");
		telemetry.update();
		waitForStart();
		runtime.reset();
		servoUpdateTimer.reset();
		// Run until the end of the match (driver presses STOP)
		while (opModeIsActive()) {
			checkGamepad();
			if (isGoingBackward == 1) {
				sl.setPower(-1);
				sr.setPower(-1);
				intake.setPower(-1);
				intakestage2.setPower(-1);
			} else {
				IntakeUpdate();
				launcherSetSpeedFunc();
			}
			// POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
			// Note: pushing stick forward gives negative value
			axial = gamepad1.left_stick_y;
			lateral = -gamepad1.left_stick_x;
			yaw = gamepad1.right_stick_x;
			// Combine the joystick requests for each axis-motion to determine each wheel's power.
			// Set up a variable for each drive wheel to save the power level for telemetry.
			frontLeftPower = axial + lateral + yaw;
			frontRightPower = (axial - lateral) - yaw;
			backLeftPower = (axial - lateral) + yaw;
			backRightPower = (axial + lateral) - yaw;
			// Normalize the values so no wheel power exceeds 100%
			// This ensures that the robot maintains the desired motion.
			max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(frontLeftPower), Math.abs(frontRightPower), Math.abs(backLeftPower), Math.abs(backRightPower)));
			if (max > 1) {
				frontLeftPower = (float) (frontLeftPower / max);
				frontRightPower = (float) (frontRightPower / max);
				backLeftPower = (float) (backLeftPower / max);
				backRightPower = (float) (backRightPower / max);
			}
			// Send calculated power to wheels.
			fr.setPower(frontLeftPower);
			br.setPower(frontRightPower);
			fl.setPower(backLeftPower);
			bl.setPower(backRightPower);
			// Show the elapsed game time and wheel power.
			telemetry.addData("Status", "Run Time: " + runtime);
			telemetry.addData("Front left/Right", JavaUtil.formatNumber(frontLeftPower, 4, 2) + ", " + JavaUtil.formatNumber(frontRightPower, 4, 2));
			telemetry.addData("LauncherSpeedInt", launcherSpeedInt);
			telemetry.addData("Back  left/Right", JavaUtil.formatNumber(backLeftPower, 4, 2) + ", " + JavaUtil.formatNumber(backRightPower, 4, 2));
			telemetry.addData("GamePad Timer", gamePadTime.milliseconds());
			telemetry.addData("Launcher Velo: ", ((DcMotorEx) launcher).getVelocity(AngleUnit.DEGREES));
			telemetry.update();
		}
	}

	/**
	 * Describe this function...
	 */
	private void launcherSetSpeedFunc() {
		if (launcherSpeedInt == 0) {
			launcher.setPower(0);
			launcher2.setPower(0);
		}
		if (launcherSpeedInt == 1) {
			launcher.setPower(0.6);
			launcher2.setPower(0.6);
		}
		if (launcherSpeedInt == 2) {
			launcher.setPower(0.45);
			launcher2.setPower(0.45);
		}
		if (launcherSpeedInt == 3) {
			launcher.setPower(0.5);
			launcher2.setPower(0.5);
		}
		if (launcherSpeedInt == 4) {
			launcher.setPower(0.53);
			launcher2.setPower(0.53);
		}
		if (launcherSpeedInt == -1) {
			launcher.setPower(-1);
			launcher2.setPower(-1);
		}
	}

	/**
	 * Describe this function...
	 */
	private void IntakeUpdate() {
		if (intakebooleanrunning == 1) {
			intake.setPower(0.8);
			if (isLaunching == 1) {
				intakestage2.setPower(0.6);
			} else {
				intakestage2.setPower(0.45);
			}
		}
		if (intakebooleanrunning == -1) {
			intake.setPower(-1);
			if (isLaunching == 1) {
				intakestage2.setPower(1);
			} else {
				intakestage2.setPower(-1);
			}
		}
		if (intakebooleanrunning == 0) {
			intake.setPower(0);
			if (isLaunching == 1) {
				intakestage2.setPower(1);
			} else {
				intakestage2.setPower(0);
			}
		}
		if (Intake2Booleanrunning == 1) {
			sl.setPower(1);
			sr.setPower(1);
		}
		if (Intake2Booleanrunning == -1) {
			sl.setPower(-1);
			sr.setPower(-1);
		}
		if (Intake2Booleanrunning == 0) {
			sl.setPower(0);
			sr.setPower(0);
		}
		if (launcherSpeedInt != 0 && !(0.2 < gamepad2.right_trigger)) {
			sl.setPower(-1);
			sr.setPower(-1);
		}
	}

	/**
	 * Describe this function...
	 */
	private void do_something() {
	}
}