package org.firstinspires.ftc.teamcode.drive;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.PoseHistory;

import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;

/**
 * DashboardDrawing - Utility class for drawing robot pose and path history
 * to FTC Dashboard. Extracted from RobotCoreCustom for modularity.
 */
public class DashboardDrawing {

	/**
	 * Draw the current robot position on the dashboard.
	 * @param follower The Pedro Pathing follower to get the pose from
	 */
	public static void drawCurrent(Follower follower) {
		try {
			Drawing.drawRobot(follower.getPose());
			Drawing.sendPacket();
		} catch (Exception e) {
			throw new RuntimeException("Drawing failed " + e);
		}
	}

	/**
	 * Thread-safe version of drawCurrent that takes a pre-fetched Pose.
	 * Use this when calling from a background thread to avoid race conditions.
	 * @param pose The pre-fetched pose to draw
	 */
	public static void drawCurrentWithPose(Pose pose) {
		try {
			if (pose != null) {
				Drawing.drawRobot(pose);
				Drawing.sendPacket();
			}
		} catch (Exception e) {
			// Log but don't crash - drawing is non-critical
			System.err.println("Drawing failed: " + e.getMessage());
		}
	}

	/**
	 * Draw the current robot position and its path history on the dashboard.
	 * @param follower The Pedro Pathing follower to get pose and history from
	 */
	public static void drawCurrentAndHistory(Follower follower) {
		PoseHistory poseHistory = follower.getPoseHistory();
		Drawing.drawPoseHistory(poseHistory);
		drawCurrent(follower);
	}
}

