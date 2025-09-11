package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.List;

/**
 * AprilTag Localizer Class for FTC
 *
 * This class provides real-time robot localization using AprilTag detection.
 * It can be easily integrated into any OpMode for autonomous navigation.
 *
 * Usage:
 *   AprilTagLocalizer localizer = new AprilTagLocalizer(hardwareMap, telemetry);
 *   localizer.init();
 *
 *   // In your loop:
 *   localizer.update();
 *   double x = localizer.getRobotX();
 *   double y = localizer.getRobotY();
 *   double heading = localizer.getRobotHeading();
 */
public class AprilTagLocalizer {

	// Vision components
	private VisionPortal visionPortal;
	private AprilTagProcessor aprilTag;
	private HardwareMap hardwareMap;
	private Telemetry telemetry;

	// Robot pose tracking
	private double robotX = 0.0;
	private double robotY = 0.0;
	private double robotHeading = 0.0;
	private boolean isLocalized = false;
	private long lastDetectionTime = 0;

	// Camera configuration
	private String cameraName = "webcam";
	private int cameraWidth = 1920;
	private int cameraHeight = 1020;

	// Camera calibration parameters (adjustable)
	private double cameraHeightOffset = 8.0; // inches above ground
	private double cameraForwardOffset = 6.0; // inches forward from robot center
	private double cameraSideOffset = 0.0; // inches right from robot center
	private double cameraPitch = 0.0; // degrees (positive = tilted up)

	// Localization settings
	private boolean enableTelemetry = true;
	private double confidenceThreshold = 0.8; // Minimum detection confidence

	/**
	 * Constructor
	 * @param hardwareMap The robot's hardware map
	 * @param telemetry Telemetry object for debugging output
	 */
	public AprilTagLocalizer(HardwareMap hardwareMap, Telemetry telemetry) {
		this.hardwareMap = hardwareMap;
		this.telemetry = telemetry;
	}

	/**
	 * Initialize the AprilTag detection system
	 * Call this once during OpMode initialization
	 */
	public void init() {
		// Create AprilTag processor
		aprilTag = new AprilTagProcessor.Builder()
				.setDrawAxes(true)
				.setDrawCubeProjection(true)
				.setDrawTagOutline(true)
				.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
				.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
				.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
				.build();

		// Create vision portal
		visionPortal = new VisionPortal.Builder()
				.setCamera(hardwareMap.get(WebcamName.class, cameraName))
				.setCameraResolution(new android.util.Size(cameraWidth, cameraHeight))
				.addProcessor(aprilTag)
				.build();

		if (enableTelemetry) {
			telemetry.addData("AprilTag Localizer", "Initialized");
			telemetry.update();
		}
	}

	/**
	 * Update robot localization
	 * Call this regularly in your OpMode loop
	 */
	public void update() {
		List<AprilTagDetection> currentDetections = aprilTag.getDetections();

		if (!currentDetections.isEmpty()) {
			// Find the best detection (highest confidence)
			AprilTagDetection bestDetection = getBestDetection(currentDetections);

			if (bestDetection != null && bestDetection.metadata != null) {
				updateRobotPose(bestDetection);
				isLocalized = true;
				lastDetectionTime = System.currentTimeMillis();

				if (enableTelemetry) {
					displayDetectionTelemetry(bestDetection);
				}
			}
		} else {
			// Check if we've lost localization
			if (System.currentTimeMillis() - lastDetectionTime > 2000) { // 2 seconds
				isLocalized = false;
			}

			if (enableTelemetry) {
				telemetry.addData("AprilTag", "No tags detected");
			}
		}

		if (enableTelemetry) {
			displayRobotPose();
		}
	}

	/**
	 * Find the detection with the highest confidence
	 */
	private AprilTagDetection getBestDetection(List<AprilTagDetection> detections) {
		AprilTagDetection best = null;
		double highestConfidence = confidenceThreshold;

		for (AprilTagDetection detection : detections) {
			if (detection.metadata != null) {
				// Use distance as a confidence metric (closer = more reliable)
				double confidence = 1.0 / (1.0 + Math.sqrt(
						detection.ftcPose.x * detection.ftcPose.x +
								detection.ftcPose.y * detection.ftcPose.y +
								detection.ftcPose.z * detection.ftcPose.z));

				if (confidence > highestConfidence) {
					highestConfidence = confidence;
					best = detection;
				}
			}
		}

		return best;
	}

	/**
	 * Update robot pose based on AprilTag detection
	 */
	private void updateRobotPose(AprilTagDetection detection) {
		// Get tag position in field coordinates
		double tagX = detection.metadata.fieldPosition.get(0);
		double tagY = detection.metadata.fieldPosition.get(1);
		double tagZ = detection.metadata.fieldPosition.get(2);
		double tagYaw = Math.toDegrees(detection.metadata.fieldOrientation.w); // Assuming yaw is the 3rd element

		// Get robot position relative to tag
		double relativeX = detection.ftcPose.x;
		double relativeY = detection.ftcPose.y;
		double relativeZ = detection.ftcPose.z;
		double relativeYaw = detection.ftcPose.yaw;

		// Calculate robot heading (tag yaw + relative yaw + 180 degrees)
		robotHeading = tagYaw + relativeYaw + 180.0;
		normalizeHeading();

		// Transform relative position to field coordinates
		double tagYawRad = Math.toRadians(tagYaw);
		double cosTagYaw = Math.cos(tagYawRad);
		double sinTagYaw = Math.sin(tagYawRad);

		// Robot position relative to tag in field coordinates
		double fieldRelativeX = relativeX * cosTagYaw - relativeY * sinTagYaw;
		double fieldRelativeY = relativeX * sinTagYaw + relativeY * cosTagYaw;

		// Add tag position to get robot field position
		robotX = tagX + fieldRelativeX;
		robotY = tagY + fieldRelativeY;

		// Apply camera offset corrections
		applyCameraOffsets();
	}

	/**
	 * Apply camera mounting offset corrections
	 */
	private void applyCameraOffsets() {
		double headingRad = Math.toRadians(robotHeading);
		double cosHeading = Math.cos(headingRad);
		double sinHeading = Math.sin(headingRad);

		robotX += cameraForwardOffset * cosHeading - cameraSideOffset * sinHeading;
		robotY += cameraForwardOffset * sinHeading + cameraSideOffset * cosHeading;
	}

	/**
	 * Normalize heading to [-180, 180] range
	 */
	private void normalizeHeading() {
		while (robotHeading > 180) robotHeading -= 360;
		while (robotHeading <= -180) robotHeading += 360;
	}

	/**
	 * Display detection telemetry
	 */
	private void displayDetectionTelemetry(AprilTagDetection detection) {
		double tagX = detection.metadata.fieldPosition.get(0);
		double tagY = detection.metadata.fieldPosition.get(1);

		telemetry.addData("Detected Tag", "ID %d", detection.id);
		telemetry.addData("Tag Position", "X=%.1f, Y=%.1f", tagX, tagY);
		telemetry.addData("Relative Pose", "X=%.1f, Y=%.1f, Z=%.1f",
				detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);
		telemetry.addData("Relative Angles", "Yaw=%.1f, Pitch=%.1f",
				detection.ftcPose.yaw, detection.ftcPose.pitch);
		telemetry.addData("Range/Bearing", "%.1f in, %.1f°", detection.ftcPose.range, detection.ftcPose.bearing);
	}

	/**
	 * Display current robot pose
	 */
	private void displayRobotPose() {
		telemetry.addData("Robot Position", "X=%.1f, Y=%.1f", robotX, robotY);
		telemetry.addData("Robot Heading", "%.1f°", robotHeading);
		telemetry.addData("Localized", isLocalized ? "YES" : "NO");
		telemetry.update();
	}

	/**
	 * Clean up vision resources
	 * Call this when your OpMode stops
	 */
	public void shutdown() {
		if (visionPortal != null) {
			visionPortal.close();
		}
	}

	// ====== GETTER METHODS ======

	/**
	 * Get robot X position on field (inches)
	 */
	public double getRobotX() {
		return robotX;
	}

	/**
	 * Get robot Y position on field (inches)
	 */
	public double getRobotY() {
		return robotY;
	}

	/**
	 * Get robot heading (degrees, -180 to 180)
	 */
	public double getRobotHeading() {
		return robotHeading;
	}

	/**
	 * Check if robot is currently localized
	 */
	public boolean isLocalized() {
		return isLocalized;
	}

	/**
	 * Get time of last successful detection (milliseconds)
	 */
	public long getLastDetectionTime() {
		return lastDetectionTime;
	}

	/**
	 * Get current robot pose as an array [x, y, heading]
	 */
	public double[] getRobotPose() {
		return new double[]{robotX, robotY, robotHeading};
	}

	// ====== CONFIGURATION METHODS ======

	/**
	 * Set camera configuration
	 */
	public void setCameraConfig(String name, int width, int height) {
		this.cameraName = name;
		this.cameraWidth = width;
		this.cameraHeight = height;
	}

	/**
	 * Set camera mounting offsets
	 */
	public void setCameraOffsets(double forwardOffset, double sideOffset, double heightOffset) {
		this.cameraForwardOffset = forwardOffset;
		this.cameraSideOffset = sideOffset;
		this.cameraHeightOffset = heightOffset;
	}

	/**
	 * Set initial robot position (useful for field-relative navigation)
	 */
	public void setInitialPose(double x, double y, double heading) {
		this.robotX = x;
		this.robotY = y;
		this.robotHeading = heading;
		this.isLocalized = true;
		this.lastDetectionTime = System.currentTimeMillis();
	}

	/**
	 * Enable or disable telemetry output
	 */
	public void setTelemetryEnabled(boolean enabled) {
		this.enableTelemetry = enabled;
	}

	/**
	 * Set minimum detection confidence threshold
	 */
	public void setConfidenceThreshold(double threshold) {
		this.confidenceThreshold = threshold;
	}

	// ====== UTILITY METHODS ======

	/**
	 * Calculate distance to a target position
	 */
	public double getDistanceToTarget(double targetX, double targetY) {
		double deltaX = targetX - robotX;
		double deltaY = targetY - robotY;
		return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
	}

	/**
	 * Calculate heading to a target position
	 */
	public double getHeadingToTarget(double targetX, double targetY) {
		double deltaX = targetX - robotX;
		double deltaY = targetY - robotY;
		return Math.toDegrees(Math.atan2(deltaY, deltaX));
	}

	/**
	 * Get heading error to target (normalized to [-180, 180])
	 */
	public double getHeadingError(double targetHeading) {
		double error = targetHeading - robotHeading;
		while (error > 180) error -= 360;
		while (error <= -180) error += 360;
		return error;
	}

	/**
	 * Get the field orientation (yaw) of a specific AprilTag
	 * Override this method to match your field's tag orientations
	 */
	private double getTagOrientation(int tagId) {
		// Default: All tags face forward (0 degrees)
		// Customize this based on your field setup
		switch (tagId) {
			case 1:
			case 2:
			case 3:
			case 4:
			case 5:
			case 6:
			case 7:
			case 8:
			case 9:
			case 10:
				return 0.0; // Most tags face forward
			default:
				return 0.0;
		}
	}
}