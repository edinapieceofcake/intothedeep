package edu.edina.Libraries.PurePursuit;

import com.acmerobotics.roadrunner.Vector2d;

public class Path {
    private double tgtSpeed, maxSpeed, radius, finalHeading;
    private boolean rotateToGoal = true;
    private Vector2d[] route;

    public Path(Vector2d[] route, double tgtSpeed, double maxSpeed, double radius) {
        this.route = route;
        this.tgtSpeed = tgtSpeed;
        this.maxSpeed = maxSpeed;
        this.radius = radius;
    }

    public Path(Vector2d[] route, double tgtSpeed, double maxSpeed, double radius, double finalHeading) {
        this(route, tgtSpeed, maxSpeed, radius);
        this.finalHeading = finalHeading;
        rotateToGoal = false;
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
