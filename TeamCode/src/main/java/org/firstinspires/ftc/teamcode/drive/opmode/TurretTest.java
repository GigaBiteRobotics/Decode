package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.CustomServoController;
import org.firstinspires.ftc.teamcode.modules.GamepadEventHandler;

@TeleOp(name = "TurretTest", group = "z")
public class TurretTest extends OpMode {

	CustomServoController servo0, servo1;
	GamepadEventHandler gp1Handler;
	double servoPos = 0;
	public void init() {
		gp1Handler = new GamepadEventHandler();
		servo0 = new CustomServoController(
				hardwareMap,
				new String[]{"azimuthServo0"},
				new boolean[]{false},
				false,
				new double[]{0,0,0,0},
				"foo"
		);
		servo1 = new CustomServoController(
				hardwareMap,
				new String[]{"azimuthServo1"},
				new boolean[]{false},
				false,
				new double[]{0,0,0,0},
				"bar"
		);
		gp1Handler.bindPress("turnServoFull", gp -> gp.dpad_up).addAction("set", () -> servoPos = 1);
		gp1Handler.bindPress("turnServoBack", gp -> gp.dpad_down).addAction("set", () -> servoPos = 0);
	}
	public void loop() {
		gp1Handler.update(gamepad1);
		servo0.setPosition(servoPos);
		servo1.setPosition(servoPos);
	}
}
