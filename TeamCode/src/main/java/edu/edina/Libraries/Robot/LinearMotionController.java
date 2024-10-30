package edu.edina.Libraries.Robot;

public class LinearMotionController {
    private final LinearMechanismSettings s;
    private double target, tPrev;

    public LinearMotionController(LinearMechanismSettings settings) {
        this.s = settings;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public double power(double t, double x, double v, double a) {
        double dt = t - tPrev;
        double dist = target - x;

        double xStop0 = estConstDeccelStoppingPoint(x, v, s.stopAccel0);
        double xStop1 = estConstDeccelStoppingPoint(x, v, s.stopAccel1);

        TimePoint coast = estConstPowerStoppingPoint(t, x, v, 0);
        boolean coastToStop = Math.abs(coast.x - target) < s.stopXTol
                && coast.t - t < s.stopTTol;
        boolean deccelToStop = between(target, xStop0, xStop1);

        double nextAccel;
        boolean clip;
        if (coastToStop) {
            nextAccel = 0;
            clip = false;
        } else if (deccelToStop) {
            nextAccel = -v * v / (2 * dist);
            clip = true;
        } else {
            nextAccel = sign(dist) * s.nominalAccel;
            clip = true;
        }

        if (clip) {
            double maxChangeAccel = s.maxJerk * dt;
            if (nextAccel < a - maxChangeAccel)
                nextAccel = a - maxChangeAccel;
            if (nextAccel > a + maxChangeAccel)
                nextAccel = a + maxChangeAccel;
        }

        double sg = !stopped(v) ? sign(v) : sign(dist);
        double power = s.ka * nextAccel + s.kv * v + s.ks * sg;

        tPrev = t;

        return power;
    }

    private double estConstDeccelStoppingPoint(double x, double v, double aMag) {
        if (stopped(v))
            return 0;

        double a = -sign(v) * aMag;
        double d = -v * v / (2 * a);
        return d + x;
    }

    private TimePoint estConstPowerStoppingPoint(double t0, double x0, double v0,
                                                 double stoppingPowerPct) {
        int sg = sign(v0);
        if (sg == 0)
            return new TimePoint(t0, x0);

        double a = -1 / (s.ka * s.maxSpeed);
        double b = -1 * (s.ks + stoppingPowerPct) / s.ka;
        double c = sg * v0 + b / a;
        double t = Math.log(b / (a * c)) / a; // when coasting will stop
        double d = (c * Math.exp(a * t) - c - b * t) / a; // where coasting will stop
        return new TimePoint(t0 + t, x0 + sg * d);
    }

    private static boolean between(double x, double bound0, double bound1) {
        if (bound0 < bound1) {
            return bound0 < x && x < bound1;
        } else {
            return bound1 < x && x < bound0;
        }
    }

    private static int sign(double x) {
        return x < 0 ? -1 : x > 0 ? 1 : 0;
    }

    private boolean stopped(double v) {
        return Math.abs(v) < s.stopVTol;
    }

    private static class TimePoint {
        public final double t, x;

        public TimePoint(double t, double x) {
            this.t = t;
            this.x = x;
        }
    }
}
