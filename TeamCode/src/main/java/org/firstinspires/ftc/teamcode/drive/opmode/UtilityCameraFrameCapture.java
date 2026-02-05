package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Utility: Camera Frame Capture", group = "Utility")
public class UtilityCameraFrameCapture extends LinearOpMode
{
    private VisionPortal portal;
    private int photoCount = 0;
    private boolean wasXPressed = false;

    @Override
    public void runOpMode()
    {
        // Build camera with internal back camera
        portal = new VisionPortal.Builder()
                .setCamera(BuiltinCameraDirection.BACK)
                .setCameraResolution(new Size(2304, 1296))
                .build();

        telemetry.addLine("Initializing camera...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            // Simple button press detection
            boolean xPressed = gamepad1.x;

            if (xPressed && !wasXPressed)
            {
                // Take photo
                takePhoto();
            }
            wasXPressed = xPressed;

            telemetry.addLine("=== Camera Capture ===");
            telemetry.addLine("Press X to take a photo");
            telemetry.addData("Photos taken", photoCount);
            telemetry.update();
        }

        // Clean up
        if (portal != null)
        {
            try
            {
                portal.stopStreaming();
                portal.close();
            }
            catch (Exception e)
            {
                // Ignore errors during cleanup
            }
        }
    }

    private void takePhoto()
    {
        try
        {
            String filename = "photo_" + System.currentTimeMillis() + ".png";
            portal.saveNextFrameRaw(filename);
            photoCount++;
            telemetry.addLine("Saved: " + filename);
            telemetry.update();
        }
        catch (Exception e)
        {
            telemetry.addLine("Error: " + e.getMessage());
            telemetry.update();
        }
    }
}
