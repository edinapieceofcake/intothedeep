package edu.edina.Libraries.Robot;

public class Accelerometer {
    private final BilinearFuncFitter bff;
    private double accel;

    public Accelerometer(int maxNumSamples) {
        bff = new BilinearFuncFitter(maxNumSamples);
    }

    public int getNumSamples() {
        return bff.getNumSamples();
    }

    public void sample(double t, double x) {
        bff.sample(t * t, t, x);
        BilinearFunc fit = bff.fit(false);
        if (fit != null) {
            accel = 2 * fit.beta0;
        }
    }

    public double getAccel() {
        return accel;
    }

    public void clear() {
        bff.clear();
    }
}
