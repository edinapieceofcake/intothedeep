package edu.edina.Libraries.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.RobotLog;

import edu.edina.Libraries.Robot.RobotHardwareChicago;
import edu.edina.Libraries.Robot.RobotState;

@Config
public class DriveForwardToDistanceAction implements Action {
    private static String TAG = "DriveForwardToDistanceAction";

    private RobotState robotState;
    private RobotHardwareChicago hw;
    private double distance, tolerance;

    private Vector2d tgtPoint;

    public static double POWER = 0.4;

    public DriveForwardToDistanceAction(RobotState robotState, RobotHardwareChicago hw, double distance, double tolerance) {
        this.hw = hw;
        this.robotState = robotState;
        this.distance = distance;
        this.tolerance = tolerance;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Pose2d pose2d = robotState.getCurrentPose();
        double d = robotState.getFrontDist();
        if (tgtPoint == null) {
            tgtPoint = new Vector2d(
                    pose2d.position.x + d * Math.cos(pose2d.heading.toDouble()),
                    pose2d.position.y + d * Math.sin(pose2d.heading.toDouble()));

            RobotLog.ii(TAG, "distance %.2f", d);
            hw.addPath(new Vector2d[]{pose2d.position, tgtPoint}, 0, Math.toDegrees(pose2d.heading.toDouble()));
            return false;
        } else {
            return true;
        }
    }
}