package edu.edina.Libraries;

import com.acmerobotics.roadrunner.Vector2d;

public class VectorCalc {
    public static Vector2d project(Vector2d v, Vector2d onto) {
        double o2 = onto.dot(onto);
        if (o2 == 0)
            return v;
        else
            return onto.times(v.dot(onto) / o2);
    }

    public static Vector2d normalize(Vector2d v) {
        double norm = v.norm();
        if (norm == 0) return v;
        else return v.div(norm);
    }

    public static double angleBetweenDeg(Vector2d v0, Vector2d v1) {
        double rad = Math.acos(normalize(v0).dot(normalize(v1)));
        return Math.toDegrees(rad);
    }

    public static double angleRad(Vector2d v) {
        return VectorCalc.normalize(v).angleCast().toDouble();
    }
}