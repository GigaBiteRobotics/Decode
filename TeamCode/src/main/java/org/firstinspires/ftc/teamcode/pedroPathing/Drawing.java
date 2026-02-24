package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;

/**
 * This is the Drawing class. It handles the drawing of stuff on FTC Dashboard, like the robot.
 *
 * @author Lazar - 19234
 * @version 2.0 - Converted from Panels to FTC Dashboard
 */
public class Drawing {
    public static final double ROBOT_RADIUS = 9;

    private static final String ROBOT_COLOR = "#3F51B5";
    private static final String HISTORY_COLOR = "#4CAF50";

    private static TelemetryPacket currentPacket = new TelemetryPacket();

    /**
     * This prepares FTC Dashboard for using Pedro Offsets
     */
    public static void init() {
        // No offset setup needed for FTC Dashboard
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method.
     */
    public static void drawDebug(Follower follower) {
        currentPacket = new TelemetryPacket();
        Canvas canvas = currentPacket.fieldOverlay();

        if (follower.getCurrentPath() != null) {
            drawPath(canvas, follower.getCurrentPath(), ROBOT_COLOR);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(canvas, new Pose(closestPoint.getX(), closestPoint.getY(),
                    follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), ROBOT_COLOR);
        }
        drawPoseHistory(canvas, follower.getPoseHistory(), HISTORY_COLOR);
        drawRobot(canvas, follower.getPose(), HISTORY_COLOR);

        sendPacket();
    }

    /**
     * Draws a robot at a specified Pose with a specified color.
     */
    public static void drawRobot(Canvas canvas, Pose pose, String color) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        canvas.setStroke(color);
        canvas.setStrokeWidth(2);
        canvas.strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2;
        double y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent();
        double y2 = pose.getY() + v.getYComponent();

        canvas.strokeLine(x1, y1, x2, y2);
    }

    /**
     * Draws a robot at a specified Pose with default color.
     */
    public static void drawRobot(Pose pose) {
        currentPacket = new TelemetryPacket();
        Canvas canvas = currentPacket.fieldOverlay();
        drawRobot(canvas, pose, ROBOT_COLOR);
    }

    /**
     * Draws a Path with a specified color.
     */
    public static void drawPath(Canvas canvas, Path path, String color) {
        double[][] points = path.getPanelsDrawingPoints();

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        canvas.setStroke(color);
        canvas.setStrokeWidth(2);
        canvas.strokeLine(points[0][0], points[0][1], points[1][0], points[1][1]);
    }

    /**
     * Draws all the Paths in a PathChain with a specified color.
     */
    public static void drawPath(Canvas canvas, PathChain pathChain, String color) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(canvas, pathChain.getPath(i), color);
        }
    }

    /**
     * Draws the pose history of the robot.
     */
    public static void drawPoseHistory(Canvas canvas, PoseHistory poseTracker, String color) {
        canvas.setStroke(color);
        canvas.setStrokeWidth(1);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {
            canvas.strokeLine(
                    poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i],
                    poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]
            );
        }
    }

    /**
     * Draws the pose history of the robot with default color.
     */
    public static void drawPoseHistory(PoseHistory poseTracker) {
        Canvas canvas = currentPacket.fieldOverlay();
        drawPoseHistory(canvas, poseTracker, HISTORY_COLOR);
    }

    /**
     * Sends the current packet to FTC Dashboard.
     */
    public static void sendPacket() {
        FtcDashboard.getInstance().sendTelemetryPacket(currentPacket);
        currentPacket = new TelemetryPacket();
    }
}
