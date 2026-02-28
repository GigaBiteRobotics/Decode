package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 * GamepadEventHandler — A centralized event-driven input system for FTC gamepads.
 *
 * <p>Register named button/trigger bindings with different event types (press, release, hold,
 * debounced press) during init, then call {@link #update(Gamepad)} once per loop to dispatch
 * all registered callbacks automatically.</p>
 *
 * <h3>Supported Event Types:</h3>
 * <ul>
 *   <li><b>ON_PRESS</b> — fires once on the rising edge (button just pressed)</li>
 *   <li><b>ON_RELEASE</b> — fires once on the falling edge (button just released)</li>
 *   <li><b>ON_HOLD</b> — fires every loop iteration while the button is held</li>
 *   <li><b>DEBOUNCED_PRESS</b> — fires repeatedly while held, gated by a cooldown timer</li>
 * </ul>
 *
 * <h3>Usage Example:</h3>
 * <pre>{@code
 * GamepadEventHandler gp2 = new GamepadEventHandler();
 *
 * // Toggle launcher on D-pad down press
 * gp2.onPress("launcherToggle", gp -> gp.dpad_down, () -> launcher.toggleSpin());
 *
 * // Adjust azimuth with 150ms debounce
 * gp2.onDebouncedPress("azimuthRight", gp -> gp.dpad_right, 150, () -> turret.adjustAzimuth(1.0));
 *
 * // Analog trigger edge detection
 * gp2.onAnalogPress("rapidFire", gp -> gp.right_trigger, 0.5, () -> launcher.startRapidFire());
 *
 * // In loop():
 * gp2.update(gamepad2);
 * }</pre>
 *
 * <p><b>Thread Safety:</b> {@code update()} must only be called from the main OpMode loop thread.</p>
 */
public class GamepadEventHandler {

    // ===== Functional Interfaces =====

    /**
     * Reads a digital (boolean) value from a Gamepad.
     * Example: {@code gp -> gp.a} or {@code gp -> gp.dpad_down}
     */
    public interface ButtonReader {
        boolean read(Gamepad gp);
    }

    /**
     * Reads an analog (double) value from a Gamepad.
     * Example: {@code gp -> gp.left_trigger} or {@code gp -> gp.right_stick_y}
     */
    public interface AnalogReader {
        double read(Gamepad gp);
    }

    /**
     * An action to execute when a binding's event fires.
     * Use lambdas: {@code () -> subsystem.doSomething()}
     */
    public interface ButtonAction {
        void run();
    }

    // ===== Event Types =====

    /**
     * Defines when a binding's action is triggered.
     */
    public enum EventType {
        /** Fires once when the button transitions from not-pressed to pressed (rising edge). */
        ON_PRESS,
        /** Fires once when the button transitions from pressed to not-pressed (falling edge). */
        ON_RELEASE,
        /** Fires every loop iteration while the button is held down. */
        ON_HOLD,
        /** Fires while the button is held, gated by a cooldown timer (repeating debounce). */
        DEBOUNCED_PRESS
    }

    // ===== Binding =====

    /**
     * Internal representation of a registered button binding.
     */
    private static class Binding {
        final String name;
        final ButtonReader reader;
        final EventType eventType;
        final ButtonAction action;
        final long debounceMs;

        // Internal state
        boolean previousState = false;
        final ElapsedTime timer = new ElapsedTime();

        Binding(String name, ButtonReader reader, EventType eventType, ButtonAction action, long debounceMs) {
            this.name = name;
            this.reader = reader;
            this.eventType = eventType;
            this.action = action;
            this.debounceMs = debounceMs;
        }
    }

    // ===== Fields =====

    private final List<Binding> bindings = new ArrayList<>();

    // ===== Registration Methods =====

    /**
     * Register a callback that fires once when a button is first pressed (rising edge).
     *
     * @param name   A descriptive name for this binding (for debugging)
     * @param reader Lambda to read the button state, e.g. {@code gp -> gp.a}
     * @param action Lambda to execute on press, e.g. {@code () -> launcher.toggleSpin()}
     * @return this handler for chaining
     */
    public GamepadEventHandler onPress(String name, ButtonReader reader, ButtonAction action) {
        bindings.add(new Binding(name, reader, EventType.ON_PRESS, action, 0));
        return this;
    }

    /**
     * Register a callback that fires once when a button is released (falling edge).
     *
     * @param name   A descriptive name for this binding
     * @param reader Lambda to read the button state
     * @param action Lambda to execute on release
     * @return this handler for chaining
     */
    public GamepadEventHandler onRelease(String name, ButtonReader reader, ButtonAction action) {
        bindings.add(new Binding(name, reader, EventType.ON_RELEASE, action, 0));
        return this;
    }

    /**
     * Register a callback that fires every loop while a button is held down.
     *
     * @param name   A descriptive name for this binding
     * @param reader Lambda to read the button state
     * @param action Lambda to execute each loop while held
     * @return this handler for chaining
     */
    public GamepadEventHandler onHold(String name, ButtonReader reader, ButtonAction action) {
        bindings.add(new Binding(name, reader, EventType.ON_HOLD, action, 0));
        return this;
    }

    /**
     * Register a callback that fires repeatedly while held, gated by a cooldown timer.
     * The action fires immediately on first press, then again every {@code debounceMs} milliseconds.
     *
     * @param name       A descriptive name for this binding
     * @param reader     Lambda to read the button state
     * @param debounceMs Minimum milliseconds between repeated firings
     * @param action     Lambda to execute
     * @return this handler for chaining
     */
    public GamepadEventHandler onDebouncedPress(String name, ButtonReader reader, long debounceMs, ButtonAction action) {
        bindings.add(new Binding(name, reader, EventType.DEBOUNCED_PRESS, action, debounceMs));
        return this;
    }

    /**
     * Register a rising-edge callback for an analog input (trigger/stick) that crosses a threshold.
     * Behaves like {@link #onPress} but converts an analog value to digital using the threshold.
     *
     * @param name      A descriptive name for this binding
     * @param reader    Lambda to read the analog value, e.g. {@code gp -> gp.right_trigger}
     * @param threshold The value above which the input is considered "pressed" (e.g. 0.5)
     * @param action    Lambda to execute on press
     * @return this handler for chaining
     */
    public GamepadEventHandler onAnalogPress(String name, AnalogReader reader, double threshold, ButtonAction action) {
        ButtonReader digitalReader = gp -> reader.read(gp) > threshold;
        bindings.add(new Binding(name, digitalReader, EventType.ON_PRESS, action, 0));
        return this;
    }

    /**
     * Register a release callback for an analog input that drops below a threshold.
     *
     * @param name      A descriptive name for this binding
     * @param reader    Lambda to read the analog value
     * @param threshold The value above which the input is considered "pressed"
     * @param action    Lambda to execute on release (when value drops below threshold)
     * @return this handler for chaining
     */
    public GamepadEventHandler onAnalogRelease(String name, AnalogReader reader, double threshold, ButtonAction action) {
        ButtonReader digitalReader = gp -> reader.read(gp) > threshold;
        bindings.add(new Binding(name, digitalReader, EventType.ON_RELEASE, action, 0));
        return this;
    }

    /**
     * Register a hold callback for an analog input above a threshold.
     *
     * @param name      A descriptive name for this binding
     * @param reader    Lambda to read the analog value
     * @param threshold The value above which the input is considered "pressed"
     * @param action    Lambda to execute each loop while above threshold
     * @return this handler for chaining
     */
    public GamepadEventHandler onAnalogHold(String name, AnalogReader reader, double threshold, ButtonAction action) {
        ButtonReader digitalReader = gp -> reader.read(gp) > threshold;
        bindings.add(new Binding(name, digitalReader, EventType.ON_HOLD, action, 0));
        return this;
    }

    /**
     * Register a debounced callback for an analog input above a threshold.
     *
     * @param name       A descriptive name for this binding
     * @param reader     Lambda to read the analog value
     * @param threshold  The value above which the input is considered "pressed"
     * @param debounceMs Minimum milliseconds between repeated firings
     * @param action     Lambda to execute
     * @return this handler for chaining
     */
    public GamepadEventHandler onAnalogDebouncedPress(String name, AnalogReader reader, double threshold, long debounceMs, ButtonAction action) {
        ButtonReader digitalReader = gp -> reader.read(gp) > threshold;
        bindings.add(new Binding(name, digitalReader, EventType.DEBOUNCED_PRESS, action, debounceMs));
        return this;
    }

    // ===== Core Update =====

    /**
     * Process all registered bindings against the current gamepad state.
     * Call this exactly once per loop iteration.
     *
     * @param gamepad The gamepad to read inputs from (gamepad1 or gamepad2)
     */
    public void update(Gamepad gamepad) {
        for (Binding binding : bindings) {
            boolean current = binding.reader.read(gamepad);

            switch (binding.eventType) {
                case ON_PRESS:
                    if (current && !binding.previousState) {
                        binding.action.run();
                    }
                    break;

                case ON_RELEASE:
                    if (!current && binding.previousState) {
                        binding.action.run();
                    }
                    break;

                case ON_HOLD:
                    if (current) {
                        binding.action.run();
                    }
                    break;

                case DEBOUNCED_PRESS:
                    if (current && binding.timer.milliseconds() > binding.debounceMs) {
                        binding.action.run();
                        binding.timer.reset();
                    }
                    break;
            }

            binding.previousState = current;
        }
    }

    // ===== Utility =====

    /**
     * Remove all registered bindings. Useful for reconfiguring controls at runtime.
     */
    public void clearAll() {
        bindings.clear();
    }

    /**
     * Get the number of registered bindings.
     * @return binding count
     */
    public int getBindingCount() {
        return bindings.size();
    }
}

