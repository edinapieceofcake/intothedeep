package edu.edina.Libraries.Robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

public interface Odometry {
    void update();

    Pose2d getPoseEstimate();

    PoseVelocity2d getVelocityEstimate();
}


