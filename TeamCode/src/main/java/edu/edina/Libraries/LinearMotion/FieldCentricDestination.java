package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class FieldCentricDestination implements Destination {
    private Pose2d relDest;

    public static double
            X_MULT = 0.1,
            Y_MULT = 0.1,
            H_MULT = 0.1;

    @Override
    public Vector2d getDestination(Pose2d pose2d) {
        return new Vector2d(pose2d.position.x + relDest.position.x, pose2d.position.y + relDest.position.y);
    }

    @Override
    public Rotation2d heading(Pose2d pose2d) {
        return pose2d.heading;
    }

    public void setRelDest(double x, double y, double h) {
        x *= X_MULT;
        y *= Y_MULT;
        h *= H_MULT;

        relDest = new Pose2d(x, y, h);
    }
}