package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

public interface Destination {
    Vector2d getDestination(Pose2d poseEst);

    Rotation2d heading(Pose2d poseEst);
}