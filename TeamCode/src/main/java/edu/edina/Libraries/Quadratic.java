package edu.edina.Libraries;

public class Quadratic {
    public static double rootOrDefault(double a, double b, double c, double defaultRoot) {
        double d = b * b - 4 * a * c;
        if (d < 0)
            return defaultRoot;

        return (-b + Math.sqrt(d)) / (2 * a);
    }
}
