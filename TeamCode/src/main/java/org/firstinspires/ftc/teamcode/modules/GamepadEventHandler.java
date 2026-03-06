package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

/**
 * GamepadEventHandler — A lambda-driven, event-based input system for FTC gamepads.
 *
 * <p>Each button/trigger binding is a single {@link GamepadBinding} variable. When the
 * binding's event condition fires (press, release, hold, debounced press), it invokes
 * all registered lambdas. Lambdas are tagged so they can be individually added, removed,
 * or replaced at runtime — multiple actions from one button press.</p>
 *
 * <h3>Key Design:</h3>
 * <ul>
 *   <li>A binding is created once via {@code handler.bind(...)}, returning a {@link GamepadBinding}</li>
 *   <li>Lambdas are added to a binding via {@code binding.addAction("tag", () -> ...)}</li>
 *   <li>Lambdas are removed via {@code binding.removeAction("tag")}</li>
 *   <li>Lambdas are replaced via {@code binding.replaceAction("tag", () -> ...)}</li>
 *   <li>Call {@link #update(Gamepad)} once per loop — the handler detects edges/holds
 *       and invokes all lambdas on triggered bindings</li>
 * </ul>
 *
 * <h3>Usage Example:</h3>
 * <pre>{@code
 * GamepadEventHandler gp2 = new GamepadEventHandler();
 *
 * // Create a binding for dpad_down press, and add two actions
 * GamepadBinding launcherBtn = gp2.bindPress("launcherToggle", gp -> gp.dpad_down);
 * launcherBtn.addAction("spin", () -> launcher.toggleSpin());
 * launcherBtn.addAction("rumble", () -> gamepad2.rumble(200));
 *
 * // Later, remove just the rumble:
 * launcherBtn.removeAction("rumble");
 *
 * // Replace the spin action with something else:
 * launcherBtn.replaceAction("spin", () -> launcher.togglePoop());
 *
 * // Debounced press with 150ms cooldown
 * GamepadBinding azBtn = gp2.bindDebouncedPress("azRight", gp -> gp.dpad_right, 150);
 * azBtn.addAction("adjust", () -> turret.adjustAzimuth(1.0));
 *
 * // Analog trigger edge detection
 * GamepadBinding rapidBtn = gp2.bindAnalogPress("rapidFire", gp -> gp.right_trigger, 0.5);
 * rapidBtn.addAction("fire", () -> launcher.startRapidFire());
 *
 * // Shorthand: bind + single action in one call (returns the binding)
 * gp2.onPress("intakeIn", gp -> gp.a, () -> intake.toggleIn());
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
     * Defines when a binding's actions are triggered.
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

    // ===== Tagged Action =====

    /**
     * A named action within a binding. The tag allows individual add/remove/replace.
     */
    private static class TaggedAction {
        final String tag;
        ButtonAction action;

        TaggedAction(String tag, ButtonAction action) {
            this.tag = tag;
            this.action = action;
        }
    }

    // ===== GamepadBinding =====

    /**
     * Represents a single button/trigger binding with an event type and a list of tagged lambdas.
     * This is the object you hold onto to add, remove, or replace actions at runtime.
     */
    public static class GamepadBinding {
        private final String name;
        private final ButtonReader reader;
        private final EventType eventType;
        private final long debounceMs;
        private final List<TaggedAction> actions = new ArrayList<>();

        // Internal edge-detection state
        boolean previousState = false;
        final ElapsedTime timer = new ElapsedTime();

        GamepadBinding(String name, ButtonReader reader, EventType eventType, long debounceMs) {
            this.name = name;
            this.reader = reader;
            this.eventType = eventType;
            this.debounceMs = debounceMs;
        }

        /**
         * Add a new tagged action to this binding. When the event fires, this action
         * will be invoked along with all other actions on this binding.
         * If a tag already exists, it is replaced.
         *
         * @param tag    Unique tag for this action (used for remove/replace)
         * @param action Lambda to execute
         * @return this binding for chaining
         */
        public GamepadBinding addAction(String tag, ButtonAction action) {
            for (TaggedAction ta : actions) {
                if (ta.tag.equals(tag)) {
                    ta.action = action;
                    return this;
                }
            }
            actions.add(new TaggedAction(tag, action));
            return this;
        }

        /**
         * Remove an action by its tag. No-op if the tag doesn't exist.
         *
         * @param tag The tag of the action to remove
         * @return this binding for chaining
         */
        public GamepadBinding removeAction(String tag) {
            Iterator<TaggedAction> it = actions.iterator();
            while (it.hasNext()) {
                if (it.next().tag.equals(tag)) {
                    it.remove();
                    break;
                }
            }
            return this;
        }

        /**
         * Replace an existing action's lambda by tag. If the tag doesn't exist, adds it.
         *
         * @param tag    The tag to find and replace
         * @param action The new lambda
         * @return this binding for chaining
         */
        public GamepadBinding replaceAction(String tag, ButtonAction action) {
            return addAction(tag, action);
        }

        /**
         * Check if an action with the given tag exists on this binding.
         *
         * @param tag The tag to look for
         * @return true if the action exists
         */
        public boolean hasAction(String tag) {
            for (TaggedAction ta : actions) {
                if (ta.tag.equals(tag)) return true;
            }
            return false;
        }

        /**
         * Remove all actions from this binding.
         *
         * @return this binding for chaining
         */
        public GamepadBinding clearActions() {
            actions.clear();
            return this;
        }

        /**
         * Get the number of actions registered on this binding.
         *
         * @return action count
         */
        public int getActionCount() {
            return actions.size();
        }

        /**
         * Get the name of this binding.
         *
         * @return binding name
         */
        public String getName() {
            return name;
        }

        /**
         * Fire all registered actions. Called internally by the handler when the event triggers.
         */
        void fireAll() {
            for (TaggedAction ta : actions) {
                ta.action.run();
            }
        }
    }

    // ===== Fields =====

    private final Map<String, GamepadBinding> bindings = new HashMap<>();

    // ===== Bind Methods (return GamepadBinding to attach lambdas) =====

    /**
     * Create a rising-edge (ON_PRESS) binding. Returns the binding so you can add actions.
     *
     * @param name   A unique name for this binding
     * @param reader Lambda to read the button state, e.g. {@code gp -> gp.a}
     * @return The created {@link GamepadBinding}
     */
    public GamepadBinding bindPress(String name, ButtonReader reader) {
        GamepadBinding binding = new GamepadBinding(name, reader, EventType.ON_PRESS, 0);
        bindings.put(name, binding);
        return binding;
    }

    /**
     * Create a falling-edge (ON_RELEASE) binding.
     *
     * @param name   A unique name for this binding
     * @param reader Lambda to read the button state
     * @return The created {@link GamepadBinding}
     */
    public GamepadBinding bindRelease(String name, ButtonReader reader) {
        GamepadBinding binding = new GamepadBinding(name, reader, EventType.ON_RELEASE, 0);
        bindings.put(name, binding);
        return binding;
    }

    /**
     * Create a hold (ON_HOLD) binding — fires every loop while held.
     *
     * @param name   A unique name for this binding
     * @param reader Lambda to read the button state
     * @return The created {@link GamepadBinding}
     */
    public GamepadBinding bindHold(String name, ButtonReader reader) {
        GamepadBinding binding = new GamepadBinding(name, reader, EventType.ON_HOLD, 0);
        bindings.put(name, binding);
        return binding;
    }

    /**
     * Create a debounced press binding — fires repeatedly while held, gated by cooldown.
     *
     * @param name       A unique name for this binding
     * @param reader     Lambda to read the button state
     * @param debounceMs Minimum milliseconds between repeated firings
     * @return The created {@link GamepadBinding}
     */
    public GamepadBinding bindDebouncedPress(String name, ButtonReader reader, long debounceMs) {
        GamepadBinding binding = new GamepadBinding(name, reader, EventType.DEBOUNCED_PRESS, debounceMs);
        bindings.put(name, binding);
        return binding;
    }

    /**
     * Create a rising-edge binding for an analog input that crosses a threshold.
     *
     * @param name      A unique name for this binding
     * @param reader    Lambda to read the analog value, e.g. {@code gp -> gp.right_trigger}
     * @param threshold The value above which the input is considered "pressed"
     * @return The created {@link GamepadBinding}
     */
    public GamepadBinding bindAnalogPress(String name, AnalogReader reader, double threshold) {
        ButtonReader digitalReader = gp -> reader.read(gp) > threshold;
        GamepadBinding binding = new GamepadBinding(name, digitalReader, EventType.ON_PRESS, 0);
        bindings.put(name, binding);
        return binding;
    }

    /**
     * Create a falling-edge binding for an analog input dropping below a threshold.
     *
     * @param name      A unique name for this binding
     * @param reader    Lambda to read the analog value
     * @param threshold The value above which the input is considered "pressed"
     * @return The created {@link GamepadBinding}
     */
    public GamepadBinding bindAnalogRelease(String name, AnalogReader reader, double threshold) {
        ButtonReader digitalReader = gp -> reader.read(gp) > threshold;
        GamepadBinding binding = new GamepadBinding(name, digitalReader, EventType.ON_RELEASE, 0);
        bindings.put(name, binding);
        return binding;
    }

    /**
     * Create a hold binding for an analog input above a threshold.
     *
     * @param name      A unique name for this binding
     * @param reader    Lambda to read the analog value
     * @param threshold The value above which the input is considered "pressed"
     * @return The created {@link GamepadBinding}
     */
    public GamepadBinding bindAnalogHold(String name, AnalogReader reader, double threshold) {
        ButtonReader digitalReader = gp -> reader.read(gp) > threshold;
        GamepadBinding binding = new GamepadBinding(name, digitalReader, EventType.ON_HOLD, 0);
        bindings.put(name, binding);
        return binding;
    }

    /**
     * Create a debounced binding for an analog input above a threshold.
     *
     * @param name       A unique name for this binding
     * @param reader     Lambda to read the analog value
     * @param threshold  The value above which the input is considered "pressed"
     * @param debounceMs Minimum milliseconds between repeated firings
     * @return The created {@link GamepadBinding}
     */
    public GamepadBinding bindAnalogDebouncedPress(String name, AnalogReader reader, double threshold, long debounceMs) {
        ButtonReader digitalReader = gp -> reader.read(gp) > threshold;
        GamepadBinding binding = new GamepadBinding(name, digitalReader, EventType.DEBOUNCED_PRESS, debounceMs);
        bindings.put(name, binding);
        return binding;
    }

    // ===== Shorthand Registration (bind + single action in one call) =====

    /**
     * Shorthand: create a press binding with a single action.
     *
     * @param name   Binding name (also used as the action tag)
     * @param reader Lambda to read the button state
     * @param action Lambda to execute on press
     * @return The created {@link GamepadBinding}
     */
    public GamepadBinding onPress(String name, ButtonReader reader, ButtonAction action) {
        return bindPress(name, reader).addAction(name, action);
    }

    /**
     * Shorthand: create a release binding with a single action.
     */
    public GamepadBinding onRelease(String name, ButtonReader reader, ButtonAction action) {
        return bindRelease(name, reader).addAction(name, action);
    }

    /**
     * Shorthand: create a hold binding with a single action.
     */
    public GamepadBinding onHold(String name, ButtonReader reader, ButtonAction action) {
        return bindHold(name, reader).addAction(name, action);
    }

    /**
     * Shorthand: create a debounced press binding with a single action.
     */
    public GamepadBinding onDebouncedPress(String name, ButtonReader reader, long debounceMs, ButtonAction action) {
        return bindDebouncedPress(name, reader, debounceMs).addAction(name, action);
    }

    /**
     * Shorthand: create an analog press binding with a single action.
     */
    public GamepadBinding onAnalogPress(String name, AnalogReader reader, double threshold, ButtonAction action) {
        return bindAnalogPress(name, reader, threshold).addAction(name, action);
    }

    /**
     * Shorthand: create an analog release binding with a single action.
     */
    public GamepadBinding onAnalogRelease(String name, AnalogReader reader, double threshold, ButtonAction action) {
        return bindAnalogRelease(name, reader, threshold).addAction(name, action);
    }

    /**
     * Shorthand: create an analog hold binding with a single action.
     */
    public GamepadBinding onAnalogHold(String name, AnalogReader reader, double threshold, ButtonAction action) {
        return bindAnalogHold(name, reader, threshold).addAction(name, action);
    }

    /**
     * Shorthand: create an analog debounced press binding with a single action.
     */
    public GamepadBinding onAnalogDebouncedPress(String name, AnalogReader reader, double threshold, long debounceMs, ButtonAction action) {
        return bindAnalogDebouncedPress(name, reader, threshold, debounceMs).addAction(name, action);
    }

    // ===== Core Update =====

    /**
     * Process all registered bindings against the current gamepad state.
     * Call this exactly once per loop iteration. Only bindings whose event condition
     * is met will have their lambdas fired.
     *
     * @param gamepad The gamepad to read inputs from (gamepad1 or gamepad2)
     */
    public void update(Gamepad gamepad) {
        for (GamepadBinding binding : bindings.values()) {
            boolean current = binding.reader.read(gamepad);

            switch (binding.eventType) {
                case ON_PRESS:
                    if (current && !binding.previousState) {
                        binding.fireAll();
                    }
                    break;

                case ON_RELEASE:
                    if (!current && binding.previousState) {
                        binding.fireAll();
                    }
                    break;

                case ON_HOLD:
                    if (current) {
                        binding.fireAll();
                    }
                    break;

                case DEBOUNCED_PRESS:
                    if (current && binding.timer.milliseconds() > binding.debounceMs) {
                        binding.fireAll();
                        binding.timer.reset();
                    }
                    break;
            }

            binding.previousState = current;
        }
    }

    // ===== Utility =====

    /**
     * Get a binding by name. Useful for adding/removing actions after initial setup.
     *
     * @param name The binding name
     * @return The binding, or null if not found
     */
    public GamepadBinding getBinding(String name) {
        return bindings.get(name);
    }

    /**
     * Remove an entire binding (and all its actions) by name.
     *
     * @param name The binding name to remove
     */
    public void removeBinding(String name) {
        bindings.remove(name);
    }

    /**
     * Remove all registered bindings. Useful for reconfiguring controls at runtime.
     */
    public void clearAll() {
        bindings.clear();
    }

    /**
     * Get the number of registered bindings.
     *
     * @return binding count
     */
    public int getBindingCount() {
        return bindings.size();
    }

    /**
     * Get the total number of actions across all bindings.
     *
     * @return total action count
     */
    public int getTotalActionCount() {
        int count = 0;
        for (GamepadBinding binding : bindings.values()) {
            count += binding.getActionCount();
        }
        return count;
    }
}
