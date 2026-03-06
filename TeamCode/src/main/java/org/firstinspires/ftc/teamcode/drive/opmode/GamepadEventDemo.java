package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.GamepadEventHandler;
import org.firstinspires.ftc.teamcode.drive.GamepadEventHandler.GamepadBinding;

/**
 * GamepadEventDemo — A simple demo showing how the lambda-driven GamepadEventHandler works.
 *
 * <p>No hardware needed. All actions just update counters/strings shown in telemetry
 * so you can see exactly when lambdas fire, and test add/remove/replace at runtime.</p>
 *
 * <h3>Controls (Gamepad 1):</h3>
 * <ul>
 *   <li><b>A</b> — Press fires two actions: "count" (increments counter) + "log" (sets message)</li>
 *   <li><b>B</b> — Press removes the "log" action from A, so only "count" remains</li>
 *   <li><b>X</b> — Press re-adds "log" back to A</li>
 *   <li><b>Y</b> — Press replaces "count" action with a decrement instead</li>
 *   <li><b>D-pad Up</b> — Debounced hold (200ms) increments a separate counter</li>
 *   <li><b>D-pad Down</b> — Clears ALL actions from A button</li>
 *   <li><b>Right Trigger</b> — Analog press (threshold 0.5) fires a trigger counter</li>
 * </ul>
 */
@Disabled
@TeleOp(name = "Gamepad Event Demo", group = "demo")
public class GamepadEventDemo extends OpMode {

    private GamepadEventHandler handler;

    // The binding variable — this is what you hold onto
    private GamepadBinding aButtonBinding;

    // State for telemetry
    private int counter = 0;
    private String lastMessage = "(none)";
    private int dpadCounter = 0;
    private int triggerCounter = 0;
    private String lastAction = "";

    @Override
    public void init() {
        handler = new GamepadEventHandler();

        // =====================================================
        // 1) CREATE A BINDING — one variable, multiple lambdas
        // =====================================================
        aButtonBinding = handler.bindPress("aButton", gp -> gp.a);

        // Attach two tagged lambdas — both fire on a single A press
        aButtonBinding.addAction("count", () -> {
            counter++;
            lastAction = "A -> count (incremented)";
        });

        aButtonBinding.addAction("log", () -> {
            lastMessage = "A was pressed! Counter is " + counter;
            lastAction = "A -> log (message set)";
        });

        // =====================================================
        // 2) B BUTTON — removes the "log" action from A
        // =====================================================
        handler.onPress("removeLog", gp -> gp.b, () -> {
            aButtonBinding.removeAction("log");
            lastAction = "B -> removed 'log' from A";
        });

        // =====================================================
        // 3) X BUTTON — re-adds "log" back to A
        // =====================================================
        handler.onPress("readdLog", gp -> gp.x, () -> {
            aButtonBinding.addAction("log", () -> {
                lastMessage = "A pressed (log re-added)! Counter=" + counter;
            });
            lastAction = "X -> re-added 'log' to A";
        });

        // =====================================================
        // 4) Y BUTTON — replaces "count" with decrement
        // =====================================================
        handler.onPress("replaceCount", gp -> gp.y, () -> {
            aButtonBinding.replaceAction("count", () -> {
                counter--;
                lastAction = "A -> count (DECREMENTED)";
            });
            lastAction = "Y -> replaced 'count' with decrement";
        });

        // =====================================================
        // 5) D-PAD UP — debounced hold (fires every 200ms while held)
        // =====================================================
        handler.onDebouncedPress("dpadHold", gp -> gp.dpad_up, 200, () -> {
            dpadCounter++;
            lastAction = "DpadUp -> debounced increment";
        });

        // =====================================================
        // 6) D-PAD DOWN — clears ALL actions from A button
        // =====================================================
        handler.onPress("clearA", gp -> gp.dpad_down, () -> {
            aButtonBinding.clearActions();
            lastAction = "DpadDown -> cleared ALL actions from A";
        });

        // =====================================================
        // 7) RIGHT TRIGGER — analog press with 0.5 threshold
        // =====================================================
        handler.onAnalogPress("triggerFire", gp -> (double) gp.right_trigger, 0.5, () -> {
            triggerCounter++;
            lastAction = "RightTrigger -> analog press fired";
        });

        telemetry.addData("Status", "Initialized — press A to start");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ONE call processes all bindings and fires matching lambdas
        handler.update(gamepad1);

        // ===== TELEMETRY =====
        telemetry.addData("--- A Button Binding ---", "");
        telemetry.addData("  Actions on A", aButtonBinding.getActionCount());
        telemetry.addData("  Has 'count'?", aButtonBinding.hasAction("count"));
        telemetry.addData("  Has 'log'?", aButtonBinding.hasAction("log"));
        telemetry.addData("  Counter", counter);
        telemetry.addData("  Message", lastMessage);

        telemetry.addData("--- Other Bindings ---", "");
        telemetry.addData("  DPad Counter (hold Up)", dpadCounter);
        telemetry.addData("  Trigger Counter", triggerCounter);

        telemetry.addData("--- Handler Stats ---", "");
        telemetry.addData("  Total Bindings", handler.getBindingCount());
        telemetry.addData("  Total Actions", handler.getTotalActionCount());

        telemetry.addData("--- Last Action ---", lastAction);

        telemetry.addData("", "");
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("A", "Fire count + log");
        telemetry.addData("B", "Remove 'log' from A");
        telemetry.addData("X", "Re-add 'log' to A");
        telemetry.addData("Y", "Replace 'count' with decrement");
        telemetry.addData("DPad Up", "Debounced hold counter");
        telemetry.addData("DPad Down", "Clear ALL actions from A");
        telemetry.addData("Right Trigger", "Analog press counter");

        telemetry.update();
    }
}

