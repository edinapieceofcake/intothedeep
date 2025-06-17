package edu.edina.Libraries.PurePursuit;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import edu.edina.Libraries.MotionControl.ICancelableAction;
import edu.edina.Libraries.Robot.Drivetrain2;
import edu.edina.Libraries.Robot.FieldToRobot;
import edu.edina.Libraries.Robot.MotionControlSettings;
import edu.edina.Libraries.Robot.RobotState;
import edu.edina.Libraries.VectorCalc;
import edu.edina.Tests.PurePursuit.MotorCommand;

public class PurePursuitAction implements ICancelableAction {
    public static double VEL_LIMIT = 16;
    public static double MAX_POWER = 0.75;
    public static double POS_TOL = 1;
    public static double VEL_TOL = 1;
    public static double P_COEFF_LIN = 1;
    public static double P_COEFF_ANG = 0.5;

    public static double AXIAL_KA = 0.0025;

    public static double AXIAL_KS = 0.066229;
    public static double AXIAL_KV = 0.015638;
    public static double LAT_KA = 0.0042431;
    public static double LAT_KS = 0.12338;
    public static double LAT_KV = 0.017618;

    private ElapsedTime etime;
    private double prevTime;
    private PurePursuit purePursuit;
    private double tgtSpeed, maxSpeed, radius;
    private RobotState state;
    private Drivetrain2 dt;
    private Vector2d vecKs, vecKv, vecKa;

    private boolean done;
    private MotorCommand mc;

    public PurePursuitAction(Vector2d[] path, double tgtSpeed, double maxSpeed, double radius, Drivetrain2 dt, RobotState state) {

        purePursuit = new PurePursuit(path, false);
        this.tgtSpeed = tgtSpeed;
        this.radius = radius;
        this.maxSpeed = maxSpeed;
        this.state = state;
        this.dt = dt;
        done = false;
        etime = new ElapsedTime();

        vecKs = new Vector2d(AXIAL_KS, LAT_KS);
        vecKv = new Vector2d(AXIAL_KV, LAT_KV);
        vecKa = new Vector2d(AXIAL_KA, LAT_KA);
    }

    private double limitMagnitude(double x, double maxMag) {
        return Math.abs(x) < maxMag ? x : maxMag * Math.signum(x);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (done) {
            return false;
        }

        double t = etime.seconds();
        double dt = t - prevTime;
        prevTime = t;

        Pose2dDual<Time> poseDual = state.getCurrentPoseDual();
        Pose2d pose = poseDual.value();
        PoseVelocity2d v = poseDual.velocity().value();
        Vector2d vRel = FieldToRobot.rotateToRobotRel(pose.heading, v.linearVel);

        purePursuit.calcNextPursuitPoint(pose.position, radius);
        Vector2d pp = purePursuit.getPursuitPoint();
        Vector2d ppRel = FieldToRobot.toRobotRel(pose, pp);

        Vector2d ppNormRel = VectorCalc.normalize(ppRel);

        //Probably not right
        double ks = ppNormRel.dot(vecKs);
        double kv = ppNormRel.dot(vecKv);
        double ka = ppNormRel.dot(vecKa);

        MotionControlSettings mcs = new MotionControlSettings(ks, kv, ka, VEL_LIMIT, MAX_POWER, POS_TOL, VEL_TOL, P_COEFF_LIN);

        double p = drivePower(dt, 0, ppRel.norm(), ppNormRel.dot(vRel), mcs);

        Vector2d linearPower = ppNormRel.times(p);
        Rotation2d r = ppNormRel.angleCast();

        double yaw = r.toDouble() * P_COEFF_ANG;

        this.dt.update(linearPower.x, linearPower.y, yaw);

        return !done;
    }

    private double drivePower(double dt, double x, double targetPos, double v, MotionControlSettings settings) {
        if (Math.abs(x - targetPos) < settings.posTolerance && Math.abs(v - tgtSpeed) < settings.velTolerance) {
            done = true;
            return 0;
        }

        double dist = targetPos - x;
        int s = (int) Math.signum(dist);
        double a0 = -s * settings.accelLimit;

        double vd = tgtSpeed;
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

        return p;
    }

    @Override
    public void cancel() {
        done = true;
    }
}
