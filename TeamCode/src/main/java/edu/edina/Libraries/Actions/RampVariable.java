package edu.edina.Libraries.Actions;

import com.qualcomm.robotcore.util.ElapsedTime;

public class RampVariable {
    private final double mult, min, max;
    private ElapsedTime t;

    public RampVariable(double mult, double min, double max) {
        this.mult = mult;
        this.min = min;
        this.max = max;
    }

    public double getCurrValue() {
        if (t == null)
            t = new ElapsedTime();

        double x = mult * t.seconds();

        if (x < min)
            x = min;

        if (x > max)
            x = max;

        return x;
    }
}