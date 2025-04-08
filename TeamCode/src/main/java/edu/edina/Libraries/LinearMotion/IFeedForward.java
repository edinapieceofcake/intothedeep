package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;

public interface IFeedForward {
    double getAcceleration(DualNum<Time> posVel);
}