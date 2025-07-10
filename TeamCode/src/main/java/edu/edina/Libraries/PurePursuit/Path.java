package edu.edina.Libraries.PurePursuit;

import com.acmerobotics.roadrunner.Vector2d;

public class Path {
    private final double tgtSpeed, maxSpeed, radius, finalHeading;
    private boolean rotateToGoal = true;
    private final String name;
    private final Vector2d[] route;

    public Path(Vector2d[] route, double tgtSpeed, double maxSpeed, double radius) {
        this(route, tgtSpeed, maxSpeed, radius, 0, true, null);
    }

    public Path(Vector2d[] route, double tgtSpeed, double maxSpeed, double radius, double finalHeading) {
        this(route, tgtSpeed, maxSpeed, radius, finalHeading, false, null);
    }

    public Path(Vector2d[] route, double tgtSpeed, double maxSpeed, double radius, double finalHeading, boolean rotateToGoal, String name) {
        this.route = route;
        this.tgtSpeed = tgtSpeed;
        this.maxSpeed = maxSpeed;
        this.radius = radius;

        this.finalHeading = finalHeading;
        this.rotateToGoal = rotateToGoal;
        this.name = name;
    }

    public boolean isRotateToGoal() {
        return rotateToGoal;
    }

    public double getTgtSpeed() {
        return tgtSpeed;
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public double getRadius() {
        return radius;
    }

    public double getFinalHeadingDeg() {
        return finalHeading;
    }

    public Vector2d[] getRoute() {
        return route;
    }
}
