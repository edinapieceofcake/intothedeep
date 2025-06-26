package edu.edina.Libraries;

import java.util.ArrayList;

import edu.edina.Libraries.Robot.LinearFunc;
import edu.edina.Libraries.Robot.LinearFuncFitter;

public class NoisyFuncGuesser {
    private final double targetY;
    private final double[] x;
    private final ArrayList<Double> xList, yList;
    private int index;

    public NoisyFuncGuesser(double targetY, double minX, double maxX, int n) {
        this.targetY = targetY;
        x = new double[n];
        xList = new ArrayList<>();
        yList = new ArrayList<>();

        for (int i = 0; i < n; i++)
            x[i] = minX + (maxX - minX) * ((double) i / (n - 1));
    }

    public void eval(double x, double y) {
        xList.add(x);
        yList.add(y - targetY);
    }

    public boolean hasResult() {
        return index >= x.length;
    }

    public double getResult() {
        LinearFuncFitter lff = new LinearFuncFitter(xList, yList);
        LinearFunc f = lff.fit();
        if (f == null)
            throw new RuntimeException("could not fit function");

        return -f.alpha / f.beta;
    }

    public double getGuess() {
        return x[index++];
    }
}

