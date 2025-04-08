package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;

import edu.edina.Libraries.Actions.RunToPositionAction;

// a mechanism which moves like F=ma
// examples are arms and lifts or 1-D driving
public interface ILinearMechanism {
    void setPower(double power);

    double getPosition(boolean raw);

    DualNum<Time> getPositionAndVelocity(boolean raw);

    LinearMechanismSettings getSettings();

    default void registerAction(RunToPositionAction a) {

    }
}