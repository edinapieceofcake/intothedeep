package edu.edina.Libraries.PurePursuit;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Vector2d;

import java.util.*;

public class PurePursuit {
    Vector2d[] path;
    Intersection pursuitPoint;
    private boolean closed;

    public PurePursuit(Vector2d[] path, boolean closed) {
        this.path = path;
        this.closed = closed;
        pursuitPoint = new Intersection(0, path[0].x, path[0].y);
    }

    public Vector2d getPursuitPoint() {
        return new Vector2d(pursuitPoint.x, pursuitPoint.y);
    }

    public Vector2d[] getPath() {
        return path;
    }

    public void calcNextPursuitPoint(Vector2d location, double radius) {
        double cx = location.x;
        double cy = location.y;

        double fx = path[path.length - 1].x;
        double fy = path[path.length - 1].y;

        if (!closed) {
            if ((cx - fx) * (cx - fx) + (cy - fy) * (cy - fy) < radius * radius) {
                setEndpoint();
                return;
            }
        }

        boolean didUpdate = nextPursuit(location, radius, pursuitPoint.t, pursuitPoint.t + 1);

        if (!didUpdate) {
            boolean didUpdate2 = nextPursuit(location, radius, pursuitPoint.t, path.length);
            if (!didUpdate2) {
                nextPursuit(location, radius, 0, path.length);
            }
        }
    }

    @NonNull
    @SuppressLint("DefaultLocale")
    @Override
    public String toString() {
        StringBuilder s = new StringBuilder();
        for (Vector2d v : path) {
            if (s.length() > 0)
                s.append(", ");

            s.append(String.format("(%.1f, %.1f)", v.x, v.y));
        }

        return s.toString();
    }

    private boolean nextPursuit(Vector2d location, double radius, double minT, double maxT) {
        double cx = location.x;
        double cy = location.y;

        boolean didUpdate = false;

        for (int i = 0; i < (path.length - 1); i++) {
            Vector2d v0 = path[i];
            Vector2d v1 = path[i + 1];

            List<Intersection> isects = intersections(v0.x, v0.y, v1.x, v1.y, cx, cy, radius);
            for (Intersection segmentInt : isects) {
                Intersection p = new Intersection(segmentInt.t + i, segmentInt.x, segmentInt.y);
                if (p.t >= minT && p.t <= maxT) {
                    didUpdate = true;
                    pursuitPoint = p;
                }
            }
        }

        return didUpdate;
    }

    private void setEndpoint() {
        Vector2d lastPoint = path[path.length - 1];
        pursuitPoint = new Intersection(path.length, lastPoint.x, lastPoint.y);
    }

    private static List<Intersection> intersections(double x0, double y0, double x1, double y1, double cx, double cy,
                                                    double r) {
        ArrayList<Intersection> ilist = new ArrayList<>();

        double ux = x1 - x0;
        double vx = x0 - cx;
        double uy = y1 - y0;
        double vy = y0 - cy;

        double a = ux * ux + uy * uy;
        double b = 2 * (ux * vx + uy * vy);
        double c = vx * vx + vy * vy - r * r;
        for (double t : quadraticRoot(a, b, c)) {
            if (0 < t && t <= 1) {
                double ix = x0 * (1 - t) + x1 * t;
                double iy = y0 * (1 - t) + y1 * t;
                ilist.add(new Intersection(t, ix, iy));
            }
        }
        return ilist;
    }

    private static double[] quadraticRoot(double a, double b, double c) {
        double d = Math.pow(b, 2) - (4 * a * c);

        double x1 = (-b + Math.sqrt(d)) / (2.0 * a);
        double x2 = (-b - Math.sqrt(d)) / (2.0 * a);

        if (d < 0) {
            return new double[0];
        } else if (d > 0) {
            double[] roots = new double[]{x1, x2};
            Arrays.sort(roots);
            return roots;
        } else {
            return new double[]{x1};
        }
    }
}