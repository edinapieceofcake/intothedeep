package edu.edina.Libraries.Robot;

import android.annotation.SuppressLint;
import android.gesture.OrientedBoundingBox;

import androidx.annotation.NonNull;

import java.util.ArrayList;
import java.util.List;

public class LinearFuncFitter {
    private final int numSamples;
    private final ArrayList<Double> xs, ys;
    private int n;

    public LinearFuncFitter(List<Double> xs, List<Double> ys) {
        numSamples = Math.min(xs.size(), ys.size());
        this.xs = new ArrayList<>(xs);
        this.ys = new ArrayList<>(ys);
        n = numSamples;
    }

    public LinearFuncFitter(int numSamples) {
        this.numSamples = numSamples;
        xs = new ArrayList<Double>();
        ys = new ArrayList<Double>();
    }

    public void sample(double x, double y) {
        push(xs, x);
        push(ys, y);
        n = Math.min(xs.size(), ys.size());
    }

    public void clear() {
        xs.clear();
        ys.clear();
    }

    public int getNumSamples() {
        return n;
    }

    public LinearFunc fit() {
        if (n == 0) {
            return null;
        }

        // find average
        double xSum = 0.0;
        double ySum = 0.0;
        for (int i = 0; i < n; i++) {
            xSum += xs.get(i);
            ySum += ys.get(i);
        }

        double xMean = xSum / n;
        double yMean = ySum / n;

        // calculate variance

        double cov = 0.0;
        double xVar = 0.0;
        for (int i = 0; i < n; i++) {
            double xc = xs.get(i) - xMean;
            double yc = ys.get(i) - yMean;

            cov += xc * yc;
            xVar += xc * xc;
        }

        // do not divide by zero
        if (xVar == 0) {
            return null;
        }

        double beta = cov / xVar;
        double alpha = yMean - beta * xMean;

        //calculate R^2
        double rss = 0.0;
        double tss = 0.0;
        for (int i = 0; i < n; i++) {
            double yHat = beta * xs.get(i) + alpha;
            double res = ys.get(i) - yHat;
            double t = ys.get(i) - yMean;

            rss += res * res;
            tss += t * t;
        }

        double r2 = 1 - rss / tss;

        return new LinearFunc(beta, alpha, r2);
    }

    @SuppressLint("DefaultLocale")
    @NonNull
    @Override
    public String toString() {
        String s = "";
        for (int i = 0; i < Math.min(xs.size(), ys.size()); i++) {
            s += String.format(" (%.2f,%.2f)", xs.get(i), ys.get(i));
        }

        return s;
    }

    private void push(ArrayList<Double> list, double val) {
        list.add(val);
        while (list.size() > numSamples)
            list.remove(0);
    }
}