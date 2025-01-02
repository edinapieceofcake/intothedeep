package edu.edina.Libraries.LinearMotion;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

public class LinearMotionController {
    private static final boolean LOG = true;

    private final ILinearMechanism linearMech;
    private final IAmbientForce ambForce;
    private final LinearMechanismSettings s;
    private final ElapsedTime et;
    private final String tag;
    private double target, tPrev;
    private DualNum<Time> u;
    private double maxPower;

    public LinearMotionController(ILinearMechanism linearMech) {
        this(linearMech, null);
    }

    public LinearMotionController(ILinearMechanism linearMech, IAmbientForce ambForce) {
        this.ambForce = ambForce;
        this.linearMech = linearMech;
        this.s = linearMech.getSettings();
        this.tag = String.format("LMC %s", s.name).trim();

        et = new ElapsedTime();

        maxPower = 1;
    }

    public void setMaxPower(double power) {
        maxPower = power;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public DualNum<Time> lastPositionAndVelocity() {
        return u;
    }

    public boolean run() {
        double t = et.seconds();

        u = linearMech.getPositionAndVelocity(false);
        double x = u.get(0);
        double v = u.get(1);
        double a = ambForce != null ? ambForce.getAcceleration(u) : 0;
        double p = power(t, x, v, a);
        linearMech.setPower(p);

        return stopped(v) && Math.abs(x - target) < s.stopXTol;
    }

    // t: current time
    // x: current position
    // v: current velocity, assumed from back emf, will be countered up to power limit
    // a: ambient acceleration, will be countered up to power limit
    @SuppressLint("DefaultLocale")
    public double power(double t, double x, double v, double a) {
        double dt = t - tPrev;
        double dist = getDist(x);

        double counterAccel = -a;

        double xStop = estConstDeccelStoppingPoint(x, v, s.stopAccel);

        double coastPower = s.ka * counterAccel;
        TimePoint coast = estConstPowerStoppingPoint(t, x, v, coastPower);
        boolean coastToStop = Math.abs(coast.x - target) < s.stopXTol
                && coast.t - t < s.stopTTol;
        boolean deccelToStop = between(target, xStop, x);

        String ls = "";
        if (LOG)
            ls += String.format("run %s, target=%.2f x=%.2f v=%.2f xStop=%.2f coast=%.2f",
                    s.name, target, x, v, xStop, coast.x);

        double power;
        if (!coastToStop) {
            double nextAccel;
            if (deccelToStop) {
                nextAccel = -v * v / (2 * dist);

                if (LOG)
                    ls += String.format(" nominal deccel=%.2f", nextAccel);
            } else {
                nextAccel = sign(dist) * s.nominalAccel;

                if (LOG)
                    ls += String.format(" nominal accel=%.2f", nextAccel);
            }

            double maxChangeAccel = s.maxJerk * dt;
            if (nextAccel < a - maxChangeAccel)
                nextAccel = a - maxChangeAccel;
            if (nextAccel > a + maxChangeAccel)
                nextAccel = a + maxChangeAccel;

            double sg = !stopped(v) ? sign(v) : sign(dist);

            power = (s.ka * (nextAccel + counterAccel) + s.kv * v) * maxPower + s.ks * sg;

            if (LOG) {
                ls += String.format(" a=%.2f", nextAccel + counterAccel);
            }
        } else {
            power = coastPower;

            if (LOG)
                ls += " coasting to a stop";
        }

        if (LOG) {
            ls += String.format(" power=%.2f", power);
            RobotLog.ii(tag, ls);
        }

        tPrev = t;

        return power;
    }

    private double getDist(double x) {
        double dist = target - x;

        if (s.units == Units.DEGREES) {
            while (Math.abs(dist) > 180) {
                if (dist > 0)
                    dist -= 360;

                if (dist < 0)
                    dist += 360;
            }
        }

        return dist;
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
