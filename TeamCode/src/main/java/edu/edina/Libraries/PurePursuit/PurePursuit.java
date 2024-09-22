package edu.edina.Libraries.PurePursuit;

public class PurePursuit {
    public static void main(String[] args) {
        runPurePursuitTest();
    }

    private static void runPurePursuitTest() {
        Vector2d[] path = new Vector2d[] {
                new Vector2d(0, 0),
                new Vector2d(0, 3),
                new Vector2d(5, 5),
                new Vector2d(8, 5)
        };

        System.out.println("path");
        for (Vector2d pathPoint : path) {
            System.out.format("%f, %f\n", pathPoint.x, pathPoint.y);
        }

        PursuitPath pp = new PursuitPath(path);

        System.out.println("checking pursuit");
        checkPursuitPoint(pp, 0, 0, 0.0, 2.0);
        checkPursuitPoint(pp, 0, 1, 0.0, 3.0);
        checkPursuitPoint(pp, 0, 1.4, 0.6915694053324097, 3.2766277621329642);
        checkPursuitPoint(pp, 0.5, 1.3, 0.7197320314734673, 3.287892812589387);
        checkPursuitPoint(pp, 0.5, 1.2, 0.49999999999999983, 3.2);
        checkPursuitPoint(pp, 7, 4, 8, 5);
        checkPursuitPoint(pp, 100, 100, 8, 5);
    }

    private static void checkPursuitPoint(PursuitPath p, double locX, double locY, double expectPx, double expectPy) {
        Vector2d loc = new Vector2d(locX, locY);
        double radius = 2;

        System.out.format("loc: %f, %f | radius: %f | expect: %f, %f", locX, locY, radius, expectPx, expectPy);
        p.nextPursuitPoint(loc, radius);

        Vector2d next = p.getPursuitPoint();

        System.out.format(" | pursuit point: %f, %f\n", next.x, next.y);
        if (Math.abs(next.x - expectPx) > 1e-6 || Math.abs(next.y - expectPy) > 1e-6)
            throw new RuntimeException("mismatch");
    }
}