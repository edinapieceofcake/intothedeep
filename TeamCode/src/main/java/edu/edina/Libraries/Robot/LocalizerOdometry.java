package edu.edina.Libraries.Robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;

import edu.edina.Libraries.RoadRunner.Localizer;

public class LocalizerOdometry implements Odometry {
    private final Localizer localizer;
    private Pose2d pose;
    private PoseVelocity2d vel;

    public LocalizerOdometry(Localizer localizer) {
        this.localizer = localizer;
        pose = new Pose2d(new Vector2d(0, 0), 0);
        vel = new PoseVelocity2d(new Vector2d(0, 0), 0);
    }

    @Override
    public void update() {
        Twist2dDual<Time> t = localizer.update();
        pose = pose.plus(t.value());
        vel = t.velocity().value();
    }

    @Override
    public Pose2d getPoseEstimate() {
        return pose;
    }

    @Override
    public PoseVelocity2d getVelocityEstimate() {
        return vel;
    }
}
