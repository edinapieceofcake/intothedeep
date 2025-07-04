package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;

public class SpringForce implements IFeedForward {
    private double k;
    private double g;

    public SpringForce(double k, double g) {
        this.k = k;
        this.g = g;
    }

    @Override
    public double getPower(DualNum<Time> posVel) {
        return g+k*posVel.get(0);
    }
}