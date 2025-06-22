package edu.edina.Libraries.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import edu.edina.Libraries.MotionControl.ICancelableAction;
import edu.edina.Libraries.MotionControl.IMotionControlLinearMechanism;
import edu.edina.Libraries.Robot.MotionControlSettings;

public class MotionControlAction implements ICancelableAction {
    private final static String TAG = "MotionControlAction";
    private final double targetPos, targetVel;
    private final IMotionControlLinearMechanism mechanism;
    private final MotionControlSettings settings;
    private final ElapsedTime etime;
    private double prevTime, power;

    private boolean done;

    // eventually make a new version of this class

    public MotionControlAction(double targetPos, IMotionControlLinearMechanism mechanism) {
        this(targetPos, 0, mechanism);
    }

    public MotionControlAction(double targetPos, double targetVel, IMotionControlLinearMechanism mechanism) {
        this.settings = mechanism.getMotionSettings();

        //if target velocity is away from target x, target velocity is zero
        if (Math.signum(targetPos) != Math.signum(targetVel)) {
            targetVel = 0;
        }

        this.targetPos = targetPos;
        this.targetVel = targetVel;
        this.mechanism = mechanism;

        etime = new ElapsedTime();
        prevTime = etime.seconds();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!done) {
            mechanism.setCurrentAction(this);

            double t = etime.seconds();
            double dt = t - prevTime;
            prevTime = t;

            DualNum<Time> posAndVel = mechanism.getPositionAndVelocity(false);
            power = drivePower(dt, posAndVel.get(0), posAndVel.get(1));
            mechanism.setPower(power);
        }

        return !done;
    }

    @Override
    public void cancel() {
        done = true;
    }

    private double drivePower(double dt, double x, double v) {
        if (TAG != null) {
            RobotLog.ii(TAG, "%s: |x - tgt| = |%.1f - %.1f| < %.1f; |v - tgt| = |%.1f - %.1f| < %.1f",
                    mechanism.getName(),
                    x, targetPos, settings.posTolerance,
                    v, targetVel, settings.velTolerance);
        }

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

        double nextA = limitMagnitude((v1 - v) / dt * settings.pCoefficient, settings.accelLimit);
        double p = settings.ks * s + settings.kv * v + settings.ka * nextA;

        if (TAG != null) {
            RobotLog.ii(TAG, "%s: dist = %.1f, z = %.1f, v0 = %.1f, v1 = %.1f, nextA = %.1f, p = %.3f",
                    mechanism.getName(),
                    dist, z, v0, v1, nextA, p);
        }

        return p;
    }

    public double limitMagnitude(double x, double maxMag) {
        return Math.abs(x) < maxMag ? x : maxMag * Math.signum(x);
    }
}