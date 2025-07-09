package edu.edina.Libraries.PurePursuit;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import edu.edina.Libraries.MotionControl.ICancelableAction;
import edu.edina.Libraries.Robot.Drivetrain;
import edu.edina.Libraries.Robot.FieldToRobot;
import edu.edina.Libraries.Robot.MotionControlSettings;
import edu.edina.Libraries.Robot.RobotState;
import edu.edina.Libraries.VectorCalc;
import edu.edina.Tests.PurePursuit.MotorCommand;

@Config
public class PurePursuitAction implements ICancelableAction {
    private static String TAG = "PurePursuit";
    public static double VEL_LIMIT = 16;
    public static double MAX_POWER = 0.75;
    public static double POS_TOL = 1;
    public static double VEL_TOL = 1;
    public static double P_COEFF_LIN = 1;
    public static double P_COEFF_ANG = 0;

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
    private Drivetrain dt;
    private Vector2d vecKs, vecKv, vecKa;

    private Path path;

    private boolean done;
    private MotorCommand mc;

    public PurePursuitAction(Path path, Drivetrain dt, RobotState state) {
        this.path = path;
        purePursuit = new PurePursuit(path.getRoute(), false);
        this.tgtSpeed = path.getTgtSpeed();
        this.radius = path.getRadius();
        this.maxSpeed = path.getMaxSpeed();
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

        Rotation2d r = getPursuitAngle(ppNormRel);

        double yaw = Math.toDegrees(r.toDouble()) * P_COEFF_ANG;

        RobotLog.ii(TAG, "@(%.1f,%.1f/%.1f) -> (%.1f,%.1f) power: (%.1f,%.1f,%.1f) %s",
                pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()),
                pp.x, pp.y,
                linearPower.x, linearPower.y, yaw,
                done ? "done" : "");

        this.dt.update(linearPower.x, linearPower.y, yaw);

        return !done;
    }

    @NonNull
    private Rotation2d getPursuitAngle(Vector2d ppNormRel) {
        Rotation2d r;

        if (path.isRotateToGoal()) {
            r = Rotation2d.fromDouble(Math.toRadians(path.getFinalHeadingDeg()));
        } else {
            r = ppNormRel.angleCast();
        }

        return r;
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
