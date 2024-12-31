package edu.edina.Libraries.Robot;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.ArrayList;

public class BilinearFuncFitter {
    private final int maxNumSamples;
    private final ArrayList<Double> x0s, x1s, ys;

    public BilinearFuncFitter(int maxNumSamples) {
        this.maxNumSamples = maxNumSamples;
        x0s = new ArrayList<Double>();
        x1s = new ArrayList<Double>();
        ys = new ArrayList<Double>();
    }

    public void sample(double x0, double x1, double y) {
        push(x0s, x0);
        push(x1s, x1);
        push(ys, y);
    }

    public BilinearFunc fit(boolean assumeZeroIntercept) {
        if (assumeZeroIntercept)
            return fitWithZeroIntercept();
        else
            return fitWithIntercept();
    }

    public void clear() {
        x0s.clear();
        x1s.clear();
        ys.clear();
    }

    private BilinearFunc fitWithIntercept() {
        int n = getNumSamples();
        MatrixF x = new GeneralMatrixF(n, 3);
        VectorF y = VectorF.length(n);
        for (int i = 0; i < n; i++) {
            x.put(i, 0, x0s.get(i).floatValue());
            x.put(i, 1, x1s.get(i).floatValue());
            x.put(i, 2, 1);

            y.put(i, ys.get(i).floatValue());
        }

        MatrixF xt = x.transposed();
        MatrixF proj = xt.multiplied(x).inverted().multiplied(xt);

        VectorF beta = proj.multiplied(y);

        return new BilinearFunc(beta.get(0), beta.get(1), beta.get(2));
    }

    @Nullable
    private BilinearFunc fitWithZeroIntercept() {
        double x00 = 0;
        double x01 = 0;
        double x11 = 0;

        int n = getNumSamples();
        for (int i = 0; i < n; i++) {
            x00 += x0s.get(i) * x0s.get(i);
            x01 += x1s.get(i) * x0s.get(i);
            x11 += x1s.get(i) * x1s.get(i);
        }

        double det = x00 * x11 - x01 * x01;
        if (det == 0)
            return null;

        double m = 1 / det;
        double a = x11 * m;
        double b = -x01 * m;
        double c = b;
        double d = x00 * m;

        double beta0 = 0;
        double beta1 = 0;
        for (int i = 0; i < n; i++) {
            beta0 += (a * x0s.get(i) + b * x1s.get(i)) * ys.get(i);
            beta1 += (c * x0s.get(i) + d * x1s.get(i)) * ys.get(i);
        }

        return new BilinearFunc(beta0, beta1, 0);
    }

    public int getNumSamples() {
        return Math.min(x1s.size(), Math.min(x0s.size(), ys.size()));
    }

    @SuppressLint("DefaultLocale")
    @NonNull
    @Override
    public String toString() {
        String s = "";
        for (int i = 0; i < getNumSamples(); i++) {
            s += String.format(" (%.2f,%.2f,%.2f)", x0s.get(i), x1s.get(i), ys.get(i));
        }

        return s;
    }

    private void push(ArrayList<Double> x, double val) {
        x.add(val);
        while (x.size() > maxNumSamples)
            x.remove(0);
    }
}