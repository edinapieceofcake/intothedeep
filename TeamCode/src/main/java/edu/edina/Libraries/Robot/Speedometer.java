package edu.edina.Libraries.Robot;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Speedometer {
    private final LinearFuncFitter fitter;

    private final ElapsedTime elapsedTime;

    public Speedometer(int numSamples) {
        fitter = new LinearFuncFitter(numSamples);
        elapsedTime = new ElapsedTime();
    }

    private double speed;

    public double getSpeed() {
        return speed;
    }

    public int getNumSamples() {
        return fitter.getNumSamples();
    }

    public void clearSamples() {
        fitter.clear();
    }

    public void sample(double degrees) {
        fitter.sample(elapsedTime.seconds(), degrees);
        LinearFunc fit = fitter.fit();
        if (fit != null) {
            speed = fit.beta;
        }
    }
}
