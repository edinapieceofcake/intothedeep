package edu.edina.Libraries.PurePursuit;

import com.acmerobotics.roadrunner.Vector2d;

import edu.edina.Libraries.Robot.RobotDriver;

public class Path {
    private final double tgtSpeed, maxSpeed, radius, finalHeading;
    private boolean rotateToGoal = true;
    private final String name;
    private final Vector2d[] route;

    public Path(Vector2d[] route) {
        this(route, 0, RobotDriver.maxSpeed, RobotDriver.radius, 0, false, null);
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

    public Path withName(String newName) {
        return new Path(route, tgtSpeed, maxSpeed, radius, finalHeading, rotateToGoal, newName);
    }

    public Path withHeading(double newFinalHeading) {
        return new Path(route, tgtSpeed, maxSpeed, radius, newFinalHeading, true, name);
    }

    public Path rotateToGoal() {
        return new Path(route, tgtSpeed, maxSpeed, radius, 0, false, name);
    }

    public Path withMaxSpeed(double newMaxSpeed) {
        return new Path(route, tgtSpeed, newMaxSpeed, radius, finalHeading, rotateToGoal, name);
    }

    public Path withTargetSpeed(double newTargetSpeed) {
        return new Path(route, newTargetSpeed, maxSpeed, radius, finalHeading, rotateToGoal, name);
    }

    public Path withRadius(double newRadius) {
        return new Path(route, tgtSpeed, maxSpeed, newRadius, finalHeading, rotateToGoal, name);
    }

    public String getName() {
        return name;
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