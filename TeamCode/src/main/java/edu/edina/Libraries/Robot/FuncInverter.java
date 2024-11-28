package edu.edina.Libraries.Robot;

public class FuncInverter {
    private Point p0, p1, pResult;
    private final double yObjective, yTolerance;
    private final boolean verbose;

    public FuncInverter(double yObjective, double yTolerance) {
        this(yObjective, yTolerance, false);
    }

    public FuncInverter(double yObjective, double yTolerance, boolean verbose) {
        this.yObjective = yObjective;
        this.yTolerance = yTolerance;
        this.verbose = verbose;
    }

    public void eval(double x, double y) {
        if (verbose)
            System.out.printf("eval(%f) = %f\n", x, y);

        Point p = new Point(x, y);
        if (Math.abs(y - yObjective) < yTolerance) {
            pResult = p;
        } else if (p0 == null) {
            p0 = p;
        } else if (p1 == null) {
            p1 = p;
            if (cmpObj(p0.y) == cmpObj(p1.y))
                throw new RuntimeException("error bracketing zero");
        } else {
            if (cmpObj(y) != cmpObj(p0.y))
                p1 = p;
            else if (cmpObj(y) != cmpObj(p1.y))
                p0 = p;
            else
                throw new RuntimeException("error bracketing zero");
        }
    }

    @Override
    public String toString() {
        String s = "FuncInverter";
        if (p0 != null) {
            s += String.format(" (%f, %f)[%d]", p0.x, p0.y, cmpObj(p0.y));
        }
        if (p1 != null) {
            s += String.format(" (%f, %f)[%d]", p1.x, p1.y, cmpObj(p1.y));
        }

        return s;
    }

    public boolean hasResult() {
        return pResult != null;
    }

    public double getResult() {
        return pResult.x;
    }

    public double getGuess() {
        if (p0 != null && p1 != null)
            return (p0.x + p1.x) / 2;
        else
            throw new RuntimeException("need to evaluate 2 starting endpoints");
    }

    private int cmpObj(double y) {
        double d = y - yObjective;
        if (d < 0)
            return -1;
        else if (d > 0)
            return 1;
        else
            return 0;
    }

    private static class Point {
        public final double x, y;

        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
}
