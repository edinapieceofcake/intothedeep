package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;

public interface ILinearMechanism {
    void setPower(double power);

    double getPosition(boolean raw);

    DualNum<Time> getPositionAndVelocity(boolean raw);

    LinearMechanismSettings getSettings();
}

