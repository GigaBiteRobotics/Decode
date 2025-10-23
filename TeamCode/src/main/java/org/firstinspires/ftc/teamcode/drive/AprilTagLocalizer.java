package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * AprilTag Localizer Class for FTC

 * This class provides real-time robot localization using AprilTag detection.
 * It can be easily integrated into any OpMode for autonomous navigation.

 * Usage:
 *   AprilTagLocalizer localizer = new AprilTagLocalizer(hardwareMap, telemetry);
 *   localizer.init();

 *   // In your loop:
 *   localizer.update();
 *   double x = localizer.getRobotX();
 *   double y = localizer.getRobotY();
 *   double heading = localizer.getRobotHeading();
 */
public class AprilTagLocalizer {
	public Position cameraPosition = new Position(DistanceUnit.INCH,
			0, 0, 0, 0);
	public YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
			0, -90, 0, 0);
	/**
	 * The variable to store our instance of the AprilTag processor.
	 */
	public AprilTagProcessor aprilTag;

	/**
	 * The variable to store our instance of the vision portal.
	 */
	public VisionPortal visionPortal;

	public void initAprilTag(HardwareMap hardwareMap, String cameraName) {

		// Create the AprilTag processor.
		aprilTag = new AprilTagProcessor.Builder()

				// The following default settings are available to un-comment and edit as needed.
				.setDrawAxes(false)
				.setDrawCubeProjection(false)
				.setDrawTagOutline(true)
				//.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
				//.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
				//.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
				.setCameraPose(cameraPosition, cameraOrientation)

				// == CAMERA CALIBRATION ==
				// If you do not manually specify calibration parameters, the SDK will attempt
				// to load a predefined calibration for your camera.
				//.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
				// ... these parameters are fx, fy, cx, cy.

				.build();

		// Adjust Image Decimation to trade-off detection-range for detection-rate.
		// eg: Some typical detection data using a Logitech C920 WebCam
		// Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
		// Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
		// Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
		// Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
		// Note: Decimation can be changed on-the-fly to adapt during a match.
		//aprilTag.setDecimation(3);

		// Create the vision portal by using a builder.
		VisionPortal.Builder builder = new VisionPortal.Builder();

		// Set the camera (webcam vs. built-in RC phone camera).
		builder.setCamera(hardwareMap.get(WebcamName.class, cameraName));


		// Choose a camera resolution. Not all cameras support all resolutions.
		//builder.setCameraResolution(new Size(640, 480));

		// Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
		//builder.enableLiveView(true);

		// Set the stream format; MJPEG uses less bandwidth than default YUY2.
		//builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

		// Choose whether or not LiveView stops if no processors are enabled.
		// If set "true", monitor shows solid orange screen if no processors enabled.
		// If set "false", monitor shows camera view without annotations.
		//builder.setAutoStopLiveView(false);

		// Set and enable the processor.
		builder.addProcessor(aprilTag);

		// Build the Vision Portal, using the above settings.
		visionPortal = builder.build();

		// Disable or re-enable the aprilTag processor at any time.
		//visionPortal.setProcessorEnabled(aprilTag, true);

	}   // end method initAprilTag()
	public Double[] getPosition() {
		List<AprilTagDetection> currentDetections = aprilTag.getDetections();
		for (AprilTagDetection detection : currentDetections) {
			if (detection != null) {
				if (!detection.metadata.name.contains("Obelisk")) {
					return new Double[]{
							detection.robotPose.getPosition().x,
							detection.robotPose.getPosition().y,
							detection.robotPose.getPosition().z,
							detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)
					};
				}
			}
		}
		return null;
	}
	public boolean isLocalized() {
		try {
			List<AprilTagDetection> currentDetections = aprilTag.getDetections();
			for (AprilTagDetection detection : currentDetections) {
				if (detection != null) {
					if (!detection.metadata.name.contains("Obelisk")) {
						return true;
					}
				}
			}
			return false;
		} catch (Exception e) { return false; }
	}
	public double getDecisionMargin() {
		try {
			List<AprilTagDetection> currentDetections = aprilTag.getDetections();
			for (AprilTagDetection detection : currentDetections) {
				if (detection != null) {
					if (!detection.metadata.name.contains("Obelisk")) {
						return detection.decisionMargin;
					}
				}
			}
			return 0.0;
		} catch (Exception e) { return 0.0; }
	}
}