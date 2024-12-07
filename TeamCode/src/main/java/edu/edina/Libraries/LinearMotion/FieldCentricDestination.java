package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class FieldCentricDestination implements Destination {
    private boolean stopping;
    private Pose2d relDest;
    private Pose2d stopPose;

    public static double
            Y_MULT = 5,
            X_MULT = 5,
            H_MULT = 25;

    public FieldCentricDestination() {
        stopPose = new Pose2d(new Vector2d(0, 0), 0);
    }

    public boolean isStopping() {
        return stopping;
    }

    @Override
    public Vector2d getDestination(Pose2d pose2d) {
        if (!stopping)
            return new Vector2d(pose2d.position.x + relDest.position.x, pose2d.position.y + relDest.position.y);
        else
            return stopPose.position;
    }

    @Override
    public Rotation2d heading(Pose2d pose2d) {
        if (!stopping)
            return Rotation2d.fromDouble(pose2d.heading.toDouble() + relDest.heading.toDouble());
        else
            return stopPose.heading;
    }

    public void setRelDest(double y, double x, double h) {
        stopping = false;

        x *= X_MULT;
        y *= Y_MULT;
        h *= H_MULT;

        relDest = new Pose2d(x, y, h);
    }

    public void setStopPose(Pose2d stopPose) {
        stopping = true;
        this.stopPose = stopPose;
    }
}