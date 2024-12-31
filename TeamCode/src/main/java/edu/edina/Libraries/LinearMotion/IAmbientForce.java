package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;

public interface IAmbientForce {
    double getAcceleration(DualNum<Time> posVel);
}