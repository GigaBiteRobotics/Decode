package org.firstinspires.ftc.teamcode.drive;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "MainOpMode (Pedro)")
public class MainDrivePedro extends LinearOpMode {

    // Drive system
    private Follower follower;

    // Launcher motors (using CustomMotorController for RPM control)
    private RobotCoreCustom.CustomMotorController launcherController;
    private static final String[] LAUNCHER_MOTOR_NAMES = {"launcher", "launcher2"};
    private static final boolean[] LAUNCHER_REVERSE_MAP = {true, false};
    private static final double LAUNCHER_TICKS_PER_REV = 28.0; // GoBilda 5202 encoder ticks

    // Intake motors (using CustomMotorController)
    private RobotCoreCustom.CustomMotorController intakeController;
    private static final String[] INTAKE_MOTOR_NAMES = {"intake", "intake stage2"};
    private static final boolean[] INTAKE_REVERSE_MAP = {false, false};

    // Lift servos (paired servos for lift mechanism)
    private Servo liftServoLeft;
    private Servo liftServoRight;

    // Intake roller servos (continuous rotation)
    private CRServo intakeRollerRight;
    private CRServo intakeRollerLeft;

    // State tracking with better naming
    private boolean intakeRollersRunning = false;
    private boolean intakeMotorsRunning = false;
    private boolean isReverseMode = false;
    private boolean isLaunchingBall = false;
    private int launcherSpeedLevel = 0; // 0=off, 1-4=speed levels, -1=reverse

    // Timers
    private ElapsedTime gamepadDebounceTimer;
    private ElapsedTime runtime;

    /**
     * Process gamepad inputs and update state variables
     */
    private void processGamepadInputs() {
        // Intake rollers trigger (gamepad2 right trigger)
        intakeRollersRunning = gamepad2.right_trigger > 0.2;

        // Toggle intake motors (gamepad2 A button with debounce)
        if (gamepad2.a && gamepadDebounceTimer.milliseconds() > 200) {
            gamepadDebounceTimer.reset();
            intakeMotorsRunning = !intakeMotorsRunning;
        }

        // Launcher speed selection (gamepad2 d-pad)
        if (gamepad2.dpad_up && gamepadDebounceTimer.milliseconds() > 200) {
            gamepadDebounceTimer.reset();
            launcherSpeedLevel = 4;
        }
        if (gamepad2.dpad_right && gamepadDebounceTimer.milliseconds() > 200) {
            gamepadDebounceTimer.reset();
            launcherSpeedLevel = 3;
        }
        if (gamepad2.dpad_down && gamepadDebounceTimer.milliseconds() > 200) {
            gamepadDebounceTimer.reset();
            launcherSpeedLevel = 2;
        }
        if (gamepad2.dpad_left && gamepadDebounceTimer.milliseconds() > 200) {
            gamepadDebounceTimer.reset();
            launcherSpeedLevel = 1;
        }

        // Stop launcher (gamepad2 X button)
        if (gamepad2.x && gamepadDebounceTimer.milliseconds() > 200) {
            gamepadDebounceTimer.reset();
            launcherSpeedLevel = 0;
        }

        // Reverse all mode (gamepad2 B button)
        if (gamepad2.b) {
            launcherSpeedLevel = -1;
            intakeMotorsRunning = false;
            intakeRollersRunning = false;
        } else {
            // Reset reverse state if nothing else is active
            if (!intakeMotorsRunning && !intakeRollersRunning && launcherSpeedLevel <= 0) {
                launcherSpeedLevel = 0;
            }
            if (launcherSpeedLevel == -1) {
                launcherSpeedLevel = 0;
                intakeMotorsRunning = false;
                intakeRollersRunning = false;
            }
        }

        // Reverse intake mode (gamepad2 right bumper)
        isReverseMode = gamepad2.right_bumper;

        // Launch trigger (gamepad2 right trigger)
        isLaunchingBall = gamepad2.right_trigger > 0.2;

        // Lift control (gamepad1 d-pad)
        if (gamepad1.dpad_down || gamepad1.dpad_up) {
            if (gamepad1.dpad_up) {
                liftServoRight.setPosition(1);
                liftServoLeft.setPosition(0);
            } else {
                liftServoRight.setPosition(0);
                liftServoLeft.setPosition(1);
            }
        } else {
            // Hold position (center)
            liftServoLeft.setPosition(0.5);
            liftServoRight.setPosition(0.5);
        }
    }

    @Override
    public void runOpMode() {
        // Initialize follower from PedroPathing Constants
        follower = Constants.createFollower(hardwareMap);

        // Initialize launcher motor controller with PIDF from MDOConstants
        launcherController = new RobotCoreCustom.CustomMotorController(
                hardwareMap,
                LAUNCHER_MOTOR_NAMES,
                LAUNCHER_REVERSE_MAP,
                true,  // has encoder
                LAUNCHER_TICKS_PER_REV,
                MDOConstants.LauncherPIDF
        );

        // Initialize intake motor controller (no encoder needed for power control)
        intakeController = new RobotCoreCustom.CustomMotorController(
                hardwareMap,
                INTAKE_MOTOR_NAMES,
                INTAKE_REVERSE_MAP,
                false,  // no encoder
                28.0,   // ticks per rev (not used without encoder)
                null    // no PIDF for power mode
        );

        // Initialize lift servos
        liftServoRight = hardwareMap.get(Servo.class, "lift1");
        liftServoLeft = hardwareMap.get(Servo.class, "lift0");
        liftServoLeft.setDirection(Servo.Direction.REVERSE);
        liftServoRight.setDirection(Servo.Direction.REVERSE);

        // Initialize intake roller servos (continuous rotation)
        intakeRollerRight = hardwareMap.get(CRServo.class, "sr");
        intakeRollerLeft = hardwareMap.get(CRServo.class, "sl");
        intakeRollerLeft.setDirection(CRServo.Direction.FORWARD);
        intakeRollerRight.setDirection(CRServo.Direction.REVERSE);

        // Initialize timers
        runtime = new ElapsedTime();
        gamepadDebounceTimer = new ElapsedTime();
        gamepadDebounceTimer.reset();

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // Start follower for teleop
        follower.startTeleopDrive();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            processGamepadInputs();

            if (isReverseMode) {
                // Reverse all mechanisms
                intakeRollerLeft.setPower(-1);
                intakeRollerRight.setPower(-1);
                intakeController.setPower(-1);
            } else {
                updateIntake();
                updateLauncher();
            }

            // Update launcher RPM PID if in RPM mode
            launcherController.updateRPMPID();

            // Use PedroPathing follower for driving
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,  // forward/backward
                    -gamepad1.left_stick_x,  // strafe left/right
                    -gamepad1.right_stick_x, // rotation
                    true                     // robot-centric
            );
            follower.update();

            // Show telemetry
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Launcher Speed Level", launcherSpeedLevel);
            telemetry.addData("Launcher RPM", launcherController.getAverageRPM());
            telemetry.addData("Launcher Target RPM", launcherController.getTargetRPM());
            telemetry.addData("Intake Running", intakeMotorsRunning);
            telemetry.addData("Rollers Running", intakeRollersRunning);
            telemetry.addData("Position", follower.getPose());
            telemetry.update();
        }
    }

    /**
     * Update launcher motors based on current speed level
     * Uses RPM control via CustomMotorController for consistent speeds
     */
    private void updateLauncher() {
        switch (launcherSpeedLevel) {
            case 0:
                launcherController.setPower(0);
                break;
            case 1:
                // Zone 1: Low power (60%)
                launcherController.setPower(0.6);
                break;
            case 2:
                // Zone 2: Medium-low power (45%)
                launcherController.setPower(0.45);
                break;
            case 3:
                // Zone 3: Medium power (50%)
                launcherController.setPower(0.5);
                break;
            case 4:
                // Zone 4: High RPM using PID control
                launcherController.setRPM(MDOConstants.LauncherRPM);
                break;
            case -1:
                // Reverse
                launcherController.setPower(-1);
                break;
            default:
                launcherController.setPower(0);
                break;
        }
    }

    /**
     * Update intake motors and roller servos based on current state
     */
    private void updateIntake() {
        // Intake motors control
        if (intakeMotorsRunning) {
            // When launching, run intake stage 2 faster to feed balls
            if (isLaunchingBall) {
                // Set different powers for each motor in the group
                // Motor 0: intake at 80%, Motor 1: intake stage2 at 60%
                intakeController.setPower(1);
            } else {
                intakeController.setPower(1);
            }
        } else {
            // When not running intake, check if launching
            if (isLaunchingBall) {
                intakeController.setPower(1.0);
            } else {
                intakeController.setPower(0);
            }
        }

        // Intake roller servos control
        if (intakeRollersRunning) {
            intakeRollerLeft.setPower(1);
            intakeRollerRight.setPower(1);
        } else {
            intakeRollerLeft.setPower(0);
            intakeRollerRight.setPower(0);
        }

        // Auto-reverse rollers when launcher is active but not actively launching
        if (launcherSpeedLevel > 0 && !isLaunchingBall) {
            intakeRollerLeft.setPower(-1);
            intakeRollerRight.setPower(-1);
        }
    }
}







