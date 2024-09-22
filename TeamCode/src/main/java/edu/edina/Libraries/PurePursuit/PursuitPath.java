package edu.edina.Libraries.PurePursuit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class PursuitPath {
    Vector2d[] path;
    Intersection pursuitPoint;

    public PursuitPath(Vector2d[] path) {
        this.path = path;
        pursuitPoint = new Intersection(0, path[0].x, path[0].y);
    }

    public Vector2d getPursuitPoint() {
        return new Vector2d(pursuitPoint.x, pursuitPoint.y);
    }

    private void setEndpoint() {
        Vector2d lastPoint = path[path.length - 1];
        pursuitPoint = new Intersection(path.length, lastPoint.x, lastPoint.y);
    }

    public void nextPursuitPoint(Vector2d location, double radius) {
        double cx = location.x;
        double cy = location.y;

        double fx = path[path.length - 1].x;
        double fy = path[path.length - 1].y;

        if ((cx - fx) * (cx - fx) + (cy - fy) * (cy - fy) < radius * radius) {
            setEndpoint();
            return;
        }

        Intersection prevInt = pursuitPoint;
        boolean didUpdate = false;

        for (int i = 0; i < (path.length - 1); i++) {
            Vector2d v0 = path[i];
            Vector2d v1 = path[i + 1];

            List<Intersection> isects = intersections(v0.x, v0.y, v1.x, v1.y, cx, cy, radius);
            for (Intersection segmentInt : isects) {
                didUpdate = true;
                pursuitPoint = new Intersection(segmentInt.t + i, segmentInt.x, segmentInt.y);
                if (pursuitPoint.t > prevInt.t)
                    return;
            }
        }

        if (!didUpdate) {
            setEndpoint();
        }
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
            double[] roots = new double[] { x1, x2 };
            Arrays.sort(roots);
            return roots;
        } else {
            return new double[] { x1 };
        }
    }
}