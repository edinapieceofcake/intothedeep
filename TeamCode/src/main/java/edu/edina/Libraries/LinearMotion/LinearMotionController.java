package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

public class LinearMotionController {
    private final ILinearMechanism linearMech;
    private final IAmbientForce ambForce;
    private final LinearMechanismSettings s;
    private final ElapsedTime et;
    private final String tag;
    private double target, tPrev;
    private DualNum<Time> u;

    public LinearMotionController(ILinearMechanism linearMech) {
        this(linearMech, null);
    }

    public LinearMotionController(ILinearMechanism linearMech, IAmbientForce ambForce) {
        this.ambForce = ambForce;
        this.linearMech = linearMech;
        this.s = linearMech.getSettings();
        this.tag = String.format("LMC %s", s.name).trim();

        et = new ElapsedTime();
    }

    public void setTarget(double target) {
        this.target = target;
        RobotLog.ii(tag, "set linear motion target to %.2f", target);
        RobotLog.ii(tag, "settings %s", s);
    }

    public DualNum<Time> lastPositionAndVelocity() {
        return u;
    }

    public boolean run() {
        double t = et.seconds();

        u = linearMech.getPositionAndVelocity(false);
        double x = u.get(0);
        double v = u.get(1);
        double a = ambForce != null ? ambForce.getAcceleration() : 0;
        double p = power(t, x, v, a);
        linearMech.setPower(p);

        return stopped(v) && Math.abs(x - target) < s.stopXTol;
    }

    // t: current time
    // x: current position
    // v: current velocity, assumed from back emf, will be countered up to power limit
    // a: ambient acceleration, will be countered up to power limit
    public double power(double t, double x, double v, double a) {
        double dt = t - tPrev;
        double dist = target - x;

        double xStop0 = estConstDeccelStoppingPoint(x, v, s.stopAccel0);
        double xStop1 = estConstDeccelStoppingPoint(x, v, s.stopAccel1);

        TimePoint coast = estConstPowerStoppingPoint(t, x, v, 0);
        boolean coastToStop = Math.abs(coast.x - target) < s.stopXTol
                && coast.t - t < s.stopTTol;
        boolean deccelToStop = between(target, xStop0, xStop1);

        RobotLog.ii(tag, "x=%.2f target=%.2f xStop0=%.2f xStop1=%.2f coast=%.2f", x, target, xStop0, xStop1, coast.x);

        double counterAccel = -a;

        double nextAccel;
        if (deccelToStop) {
            nextAccel = -v * v / (2 * dist);
            RobotLog.ii(tag, "decelerating, nominal deccel=%.2f", nextAccel);
        } else {
            nextAccel = sign(dist) * s.nominalAccel;
            RobotLog.ii(tag, "accelerating, nominal accel=%.2f", nextAccel);
        }

        double power;
        if (!coastToStop) {
            double maxChangeAccel = s.maxJerk * dt;
            if (nextAccel < a - maxChangeAccel)
                nextAccel = a - maxChangeAccel;
            if (nextAccel > a + maxChangeAccel)
                nextAccel = a + maxChangeAccel;

            double sg = !stopped(v) ? sign(v) : sign(dist);

            power = s.ka * (nextAccel + counterAccel) + s.kv * v + s.ks * sg;

            RobotLog.ii(tag, "a=%.2f v=%.2f sg=%.2f", nextAccel + counterAccel, v, sg);
            RobotLog.ii(tag, "power=%.2f", power);
        } else {
            power = 0;
            RobotLog.ii(tag, "coasting to a stop");
        }

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
