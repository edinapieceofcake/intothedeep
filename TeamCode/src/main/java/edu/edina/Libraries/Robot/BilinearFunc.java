package edu.edina.Libraries.Robot;

public class BilinearFunc {
    public BilinearFunc(double beta0, double beta1, double alpha) {
        this.alpha = alpha;
        this.beta0 = beta0;
        this.beta1 = beta1;
    }

    public final double beta0;
    public final double beta1;
    public final double alpha;

    public double eval(double x0, double x1) {
        return beta0 * x0 + beta1 * x1 + alpha;
    }
}
