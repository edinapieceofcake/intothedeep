package edu.edina.Libraries;

public class LinearInterpolator {

    private final double[] xc, yc;

    public LinearInterpolator(double[] x, double[] y) {
        if (x.length != y.length) {
            throw new RuntimeException("data arrays should math");
        }

        if (x.length < 1) {
            throw new RuntimeException("at least one data point required");
        }

        for (int i = 0; i + 1 < x.length; i++) {
            if (x[i] > x[i + 1]) {
                throw new RuntimeException("x array must be sorted");
            }
        }

        xc = x.clone();
        yc = y.clone();
    }

    public double eval(double x) {
        if (x <= xc[0]) {
            return yc[0];
        }

        for (int i = 0; i + 1 < xc.length; i++) {
            double xl = xc[i];
            double xr = xc[i + 1];
            if (xl <= x && x <= xr) {
                double yl = yc[i];
                double yr = yc[i + 1];
                return (x - xl) / (xr - xl) * (yr - yl) + yl;
            }
        }

        return yc[yc.length - 1];
    }
}