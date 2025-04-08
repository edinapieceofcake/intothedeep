package edu.edina.Libraries.Robot;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;

import edu.edina.Libraries.LinearMotion.ArmSwingMechanism;
import edu.edina.Libraries.LinearMotion.IFeedForward;

public class ArmForces implements IFeedForward {
    public static double STOP_POWER = 0.12;

    private final double g;

    public ArmForces() {
        g = STOP_POWER / ArmSwingMechanism.KA;
    }

    @Override
    public double getAcceleration(DualNum<Time> posVel) {
        double rad = Math.toRadians(posVel.get(0));
        double mult = -Math.cos(rad);
        return g * mult;
    }
}