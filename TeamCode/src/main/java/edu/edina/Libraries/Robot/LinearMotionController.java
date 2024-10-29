package edu.edina.Libraries.Robot;

public class LinearMotionController {
    private static class Settings {
        public final double target = -60;
        public final double stopVTol = 0.1;
        public final double stopXTol = 0.5;
        public final double stopTTol = 0.1;
        public final double aStop0 = 10.5;
        public final double aStop1 = 9.0;
        public final double maxSpeed = 20;
        public final double nominalAccel = 10;
        public final double maxJerk = 100;
        public final double ks = 0.0666663961582924;
        public final double kv = 0.0501225246543856;
        public final double ka = 0.08327226389801484;
    }

    private static class Controller {
        private final Settings s;
        private double tPrev;

        public Controller(Settings settings) {
            this.s = settings;
        }

        public double power(double t, double x, double v, double a) {
            double dt = t - tPrev;
            double tgt = s.target;
            double dist = tgt - x;

            double xStop0 = estConstDeccelStoppingPoint(x, v, s.aStop0);
            double xStop1 = estConstDeccelStoppingPoint(x, v, s.aStop1);

            TimePoint coast = estConstPowerStoppingPoint(t, x, v, 0);
            boolean coastToStop = Math.abs(coast.x - tgt) < s.stopXTol
                    && coast.t - t < s.stopTTol;
            boolean deccelToStop = between(tgt, xStop0, xStop1);

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
    }

    private static class TimePoint {
        public final double t, x;

        public TimePoint(double t, double x) {
            this.t = t;
            this.x = x;
        }
    }
}
