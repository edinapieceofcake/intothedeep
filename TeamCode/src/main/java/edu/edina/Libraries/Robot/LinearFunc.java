package edu.edina.Libraries.Robot;

public class LinearFunc {
    public LinearFunc(double beta, double alpha, double r2) {
        this.alpha = alpha;
        this.beta = beta;
        R2 = r2; ;
    }

    public final double beta;
    public final double alpha;
    public final double R2;

    public double eval(double x) {
        return beta * x + alpha;
    }
}
