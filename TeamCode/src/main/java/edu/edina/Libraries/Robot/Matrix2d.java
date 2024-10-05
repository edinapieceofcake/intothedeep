package edu.edina.Libraries.Robot;

import com.acmerobotics.roadrunner.Vector2d;

public class Matrix2d {
    public final double a, b, c, d;

    public Matrix2d(double a, double b, double c, double d) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }

    public static  Matrix2d withColumns(Vector2d col0, Vector2d col1){
        return  new Matrix2d(col0.x, col1.x, col0.y, col1.y);
    }

    public Vector2d transform(Vector2d v) {
        return new Vector2d(
                a * v.x + b * v.y,
                c * v.x + d * v.y);
    }

    public Matrix2d invert() {
        double det = a * d - b * c;
        return new Matrix2d(
                d / det,
                -b / det,
                -c / det,
                a / det);
    }

    public Matrix2d mult(Matrix2d m) {
        return new Matrix2d(
                a * m.a + b * m.c,
                a * m.b + b * m.d,
                c * m.a + d * m.c,
                c * m.b + d * m.d
        );
    }

    @Override
    public String toString() {
        return String.format("[%.2f, %.2f] [%.2f, %.2f]", a, b, c, d);
    }
}