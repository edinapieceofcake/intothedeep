package edu.edina.Libraries.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.util.ElapsedTime;

import edu.edina.Libraries.LinearMotion.ILinearMechanism;
import edu.edina.Libraries.LinearMotion.LinearMechanismSettings;

// want to convert this to an action
public class MotionControlAction implements Action {
    // 3. doneTime was just for debugging the python code, and can be removed
    private final double targetPos, targetVel;
    private final MotionControlSettings settings;
    private final ElapsedTime etime;
    private double prevTime;

    private boolean done;

    // eventually make a new version of this class
    private ILinearMechanism linearMech;

    public MotionControlAction(double targetPos, double targetVel, ILinearMechanism linearMech, MotionControlSettings settings) {
        this.settings = settings;

        //if target velocity is away from target x, target velocity is zero
        if (Math.signum(targetPos) != Math.signum(targetVel)) {
            targetVel = 0;
        }

        this.targetPos = targetPos;
        this.targetVel = targetVel;
        this.linearMech = linearMech;

        etime = new ElapsedTime();
        prevTime = 0;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (done) {
            return false;
        } else {
            DualNum<Time> posAndVel = linearMech.getPositionAndVelocity(false);

            double t = etime.seconds();
            double power = drivePower(t, posAndVel.get(0), posAndVel.get(1));
            linearMech.setPower(power);
            prevTime = t;
            return true;
        }
    }

    private double drivePower(double t, double x, double v) {
        if (Math.abs(x - targetPos) < settings.posTolerance && Math.abs(v - targetVel) < settings.velTolerance) {
            done = true;
            return 0;
        }

        double dist = targetPos - x;
        int s = (int) Math.signum(dist);
        double a0 = -s * settings.accelLimit;

        double vd = targetVel;
        double z = vd * vd - 2 * a0 * dist;
        double v0;

        if (z > 0) {
            v0 = s * Math.sqrt(z);
        } else {
            // this should generally not happen
            v0 = s * settings.velLimit;
        }

        double v1 = limitMagnitude(v0, settings.velLimit);
        double dt = t - prevTime;
        double nextA = limitMagnitude((v1 - v) / dt / settings.pCoefficient, settings.accelLimit);
        double p = settings.ks * s + settings.kv * v + settings.ka * nextA;

        prevTime = t;

        return p;
    }

    public double limitMagnitude(double x, double maxMag) {
        return Math.abs(x) < maxMag ? x : maxMag * Math.signum(x);
    }
}