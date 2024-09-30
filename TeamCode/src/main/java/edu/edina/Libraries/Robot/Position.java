package edu.edina.Libraries.Robot;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Vector2d;

@SuppressLint("DefaultLocale")
public final class Position {
    public final double x, y, a;

    public Position(double x, double y, double a) {
        this.x = x;
        this.y = y;
        this.a = a;
    }

    @NonNull
    @Override
    public String toString() {
        return String.format("x=%.2f, y=%.2f heading=%.2f", x, y, a);
    }

    public double dist(Position p) {
        double x = this.x - p.x;
        double y = this.y - p.y;
        return Math.sqrt(x * x + y * y);
    }

    public Vector2d toRobotRel(Vector2d v) {
        double dx = getDx();
        double dy = getDy();
        double tx = v.x - this.x;
        double ty = v.y - this.y;
        return new Vector2d((tx * dy) - (ty * dx), (tx * dx) + (ty * dy));
    }

    public Position addRobotRel(Vector2d v) {
        double dx = getDx();
        double dy = getDy();
        double tx = dy * v.x + dx * v.y;
        double ty = -dx * v.x + dy * v.y;
        return new Position(x + tx, y + ty, a);
    }

    public double getDx() {
        return -Math.sin(Math.toRadians(a));
    }

    public double getDy() {
        return Math.cos(Math.toRadians(a));
    }
}