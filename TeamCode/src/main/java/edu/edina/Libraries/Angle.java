package edu.edina.Libraries;

public class Angle {
    public static double degreeDiff(double a0, double a1) {
        double d = a0 - a1;
        if (d > 0)
            return (d + 180.0) % 360.0 - 180.0;
        else
            return 180.0 - (180.0 - d) % 360.0;
    }

    public static double radianDiff(double a0, double a1) {
        double d = a0 - a1;
        if (d > 0)
            return (d + Math.PI) % (2 * Math.PI) - Math.PI;
        else
            return Math.PI - (Math.PI - d) % (2 * Math.PI);
    }
}
