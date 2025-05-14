package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class
FieldCentricDestination implements Destination {
    private boolean stopping;
    private Pose2d poseEst, relDest;
    private Pose2d stopPose;
    private Vector2d rectDir;
    private double angDir;

    public static double
            Y_MULT = 50,
            X_MULT = 50,
            H_MULT = 25;

    public FieldCentricDestination() {
        relDest = new Pose2d(new Vector2d(0, 0), 0);
        stopPose = new Pose2d(new Vector2d(0, 0), 0);
    }

    public boolean isStopping() {
        return stopping;
    }

    @Override
    public void updatePose(Pose2d poseEst) {
        this.poseEst = poseEst;
    }

    @Override
    public Vector2d getDestination() {
        if (stopping) {
            return stopPose.position;
        } else {
            return new Vector2d(poseEst.position.x + relDest.position.x, poseEst.position.y + relDest.position.y);
        }
    }

    @Override
    public Rotation2d getHeading() {
        if (stopping) {
            return stopPose.heading;
        } else {
            return Rotation2d.fromDouble(poseEst.heading.toDouble() + relDest.heading.toDouble());
        }
    }

    public void setDirection(double x, double y, double h) {
        stopping = false;
        rectDir = new Vector2d(x, y);
        angDir = h;
    }

    public void setStopPose(Pose2d stopPose) {
        stopping = true;
        this.stopPose = stopPose;
    }
}