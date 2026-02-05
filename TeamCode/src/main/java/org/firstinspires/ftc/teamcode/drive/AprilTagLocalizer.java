package org.firstinspires.ftc.teamcode.drive;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import com.bylazar.camerastream.PanelsCameraStream;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

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

	/**
	 * Camera Stream Processor for FTC Dashboard
	 * Captures frames and makes them available for streaming
	 */
	public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
		private final AtomicReference<Bitmap> lastFrame =
				new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

		@Override
		public void init(int width, int height, CameraCalibration calibration) {
			lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
		}

		@Override
		public Object processFrame(Mat frame, long captureTimeNanos) {
			Bitmap bitmap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
			Utils.matToBitmap(frame, bitmap);
			lastFrame.set(bitmap);
			return null;
		}

		@Override
		public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
		                        float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
			// No drawing needed
		}

		@Override
		public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
			continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
		}
	}

	public CameraStreamProcessor streamProcessor;
	public boolean streamingEnabled = false;
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
		initAprilTag(hardwareMap, cameraName, true);
	}

	public void initAprilTag(HardwareMap hardwareMap, String cameraName, boolean enableStreaming) {
		this.streamingEnabled = enableStreaming;

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

		// Add camera stream processor if enabled
		if (enableStreaming) {
			streamProcessor = new CameraStreamProcessor();
			builder.addProcessor(streamProcessor);
		}

		// Build the Vision Portal, using the above settings.
		visionPortal = builder.build();

		// Start camera stream if enabled
		if (enableStreaming && streamProcessor != null) {
			PanelsCameraStream.INSTANCE.startStream(streamProcessor, null);
		}

		// Disable or re-enable the aprilTag processor at any time.
		//visionPortal.setProcessorEnabled(aprilTag, true);

	}   // end method initAprilTag()

	private java.lang.reflect.Method setCameraPoseMethod = null;
	private boolean setCameraPoseMethodInitialized = false;

	/**
	 * Updates the camera pose (position and orientation) for the AprilTag processor.
	 * This allows for runtime adjustments of the camera offset.
	 *
	 * @param position    The new position of the camera relative to the robot center.
	 * @param orientation The new orientation of the camera.
	 */
	public void setCameraPose(Position position, YawPitchRollAngles orientation) {
		this.cameraPosition = position;
		this.cameraOrientation = orientation;
		if (aprilTag != null) {
			try {
				if (!setCameraPoseMethodInitialized) {
					try {
						// Use reflection to call setCameraPose since it might not be exposed in the interface
						setCameraPoseMethod = aprilTag.getClass().getMethod("setCameraPose", Position.class, YawPitchRollAngles.class);
					} catch (NoSuchMethodException e) {
						setCameraPoseMethod = null;
					}
					setCameraPoseMethodInitialized = true;
				}

				if (setCameraPoseMethod != null) {
					setCameraPoseMethod.invoke(aprilTag, position, orientation);
				}
			} catch (Exception e) {
				// invoke failed, ignore
			}
		}
	}

	public Double[] getPosition() {
		try {
			List<AprilTagDetection> currentDetections = aprilTag.getDetections();
			for (AprilTagDetection detection : currentDetections) {
				if (detection != null) {
					if (!detection.metadata.name.contains("Obelisk")) {
						// Filter by maximum distance if specified using ftcPose.range
						if (detection.ftcPose != null && detection.ftcPose.range > MDOConstants.AprilTagMaxDistance) {
							continue;
						}

						// Get the camera's pose (since we initialized processor with 0 offset)
						double camX = detection.robotPose.getPosition().x;
						double camY = detection.robotPose.getPosition().y;
						double camZ = detection.robotPose.getPosition().z;
						double yaw = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);

						// Calculate robot position by subtracting the rotated camera offset
						// x' = x*cos(theta) - y*sin(theta)
						// y' = x*sin(theta) + y*cos(theta)
						double xOffset = cameraPosition.x;
						double yOffset = cameraPosition.y;

						double xRotated = xOffset * Math.cos(yaw) - yOffset * Math.sin(yaw);
						double yRotated = xOffset * Math.sin(yaw) + yOffset * Math.cos(yaw);

						return new Double[]{
								camX - xRotated,
								camY - yRotated,
								camZ - cameraPosition.z,
								yaw,
								(double) detection.id  // Add AprilTag ID to help with debugging
						};
					}
				}
			}
		} catch (Exception e) { return null; }
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

	/**
	 * Stop camera streaming
	 * Call this in the stop() method of your OpMode
	 */
	public void stopStream() {
		if (streamingEnabled) {
			PanelsCameraStream.INSTANCE.stopStream();
		}
	}
}
