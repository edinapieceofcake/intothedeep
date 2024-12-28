package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

public interface Destination {
    void updatePose(Pose2d pose);

    Vector2d getDestination();

    Rotation2d getHeading();
}