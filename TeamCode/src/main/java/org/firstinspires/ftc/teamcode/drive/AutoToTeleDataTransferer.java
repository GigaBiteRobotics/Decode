package org.firstinspires.ftc.teamcode.drive;

import com.pedropathing.geometry.Pose;

/**
 * Singleton class for transferring data from Autonomous OpMode to TeleOp OpMode.
 * This class persists data across OpMode transitions.
 */
public class AutoToTeleDataTransferer {

    // Singleton instance
    private static AutoToTeleDataTransferer instance = null;

    // Data to transfer
    private Pose endPose;
    private String allianceColor;
    private String startingPosition;
    private boolean autoCompleted;
    private double autoRuntime;
    private String autoNotes;

    // Additional custom data storage
    private java.util.HashMap<String, Object> customData;

    /**
     * Private constructor for singleton pattern
     */
    private AutoToTeleDataTransferer() {
        reset();
    }

    /**
     * Get the singleton instance
     * @return The singleton instance
     */
    public static AutoToTeleDataTransferer getInstance() {
        if (instance == null) {
            instance = new AutoToTeleDataTransferer();
        }
        return instance;
    }

    /**
     * Reset all stored data to defaults
     */
    public void reset() {
        endPose = new Pose(0, 0, 0);
        allianceColor = "UNKNOWN";
        startingPosition = "UNKNOWN";
        autoCompleted = false;
        autoRuntime = 0.0;
        autoNotes = "";
        customData = new java.util.HashMap<>();
    }

    // Getters and Setters

    /**
     * Set the final pose from autonomous
     * @param pose The final pose of the robot
     */
    public void setEndPose(Pose pose) {
        this.endPose = pose;
    }

    /**
     * Get the final pose from autonomous
     * @return The final pose of the robot
     */
    public Pose getEndPose() {
        return endPose;
    }

    /**
     * Set the alliance color
     * @param color The alliance color (e.g., "RED" or "BLUE")
     */
    public void setAllianceColor(String color) {
        this.allianceColor = color;
    }

    /**
     * Get the alliance color
     * @return The alliance color
     */
    public String getAllianceColor() {
        return allianceColor;
    }

    /**
     * Set the starting position
     * @param position The starting position description
     */
    public void setStartingPosition(String position) {
        this.startingPosition = position;
    }

    /**
     * Get the starting position
     * @return The starting position description
     */
    public String getStartingPosition() {
        return startingPosition;
    }

    /**
     * Set whether autonomous completed successfully
     * @param completed True if auto completed successfully
     */
    public void setAutoCompleted(boolean completed) {
        this.autoCompleted = completed;
    }

    /**
     * Check if autonomous completed successfully
     * @return True if auto completed successfully
     */
    public boolean isAutoCompleted() {
        return autoCompleted;
    }

    /**
     * Set the autonomous runtime
     * @param runtime The total runtime in seconds
     */
    public void setAutoRuntime(double runtime) {
        this.autoRuntime = runtime;
    }

    /**
     * Get the autonomous runtime
     * @return The total runtime in seconds
     */
    public double getAutoRuntime() {
        return autoRuntime;
    }

    /**
     * Set notes from autonomous
     * @param notes Any notes or messages from auto
     */
    public void setAutoNotes(String notes) {
        this.autoNotes = notes;
    }

    /**
     * Get notes from autonomous
     * @return Any notes or messages from auto
     */
    public String getAutoNotes() {
        return autoNotes;
    }

    /**
     * Store custom data with a key
     * @param key The key to store the data under
     * @param value The value to store
     */
    public void putCustomData(String key, Object value) {
        customData.put(key, value);
    }

    /**
     * Retrieve custom data by key
     * @param key The key to retrieve
     * @return The stored value, or null if not found
     */
    public Object getCustomData(String key) {
        return customData.get(key);
    }

    /**
     * Check if custom data exists for a key
     * @param key The key to check
     * @return True if data exists for the key
     */
    public boolean hasCustomData(String key) {
        return customData.containsKey(key);
    }

    /**
     * Get a summary string of all stored data
     * @return A formatted string with all data
     */
    public String getSummary() {
        StringBuilder sb = new StringBuilder();
        sb.append("=== Auto to TeleOp Data ===\n");
        sb.append(String.format("Alliance: %s\n", allianceColor));
        sb.append(String.format("Starting Position: %s\n", startingPosition));
        sb.append(String.format("Auto Completed: %s\n", autoCompleted ? "Yes" : "No"));
        sb.append(String.format("Auto Runtime: %.2fs\n", autoRuntime));
        sb.append(String.format("End Pose: (%.2f, %.2f, %.2f°)\n",
                endPose.getX(), endPose.getY(), Math.toDegrees(endPose.getHeading())));
        if (!autoNotes.isEmpty()) {
            sb.append(String.format("Notes: %s\n", autoNotes));
        }
        if (!customData.isEmpty()) {
            sb.append(String.format("Custom Data: %d entries\n", customData.size()));
        }
        return sb.toString();
    }
}
