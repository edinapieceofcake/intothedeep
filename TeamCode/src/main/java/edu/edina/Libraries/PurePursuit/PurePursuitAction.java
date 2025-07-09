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
    private final MotionControlSettings axMcs, latMcs;

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

        axMcs = new MotionControlSettings(AXIAL_KS, AXIAL_KV, AXIAL_KA, VEL_LIMIT, MAX_POWER,
                POS_TOL, VEL_TOL,
                P_COEFF_LIN);

        latMcs = new MotionControlSettings(LAT_KS, LAT_KV, LAT_KA, VEL_LIMIT, MAX_POWER, POS_TOL,
                VEL_TOL,
                P_COEFF_LIN);
    }

    private static double limitMagnitude(double x, double maxMag) {
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

        purePursuit.calcNextPursuitPoint(pose.position, radius);
        Vector2d pp = purePursuit.getPursuitPoint();

        MotionCommand cmd = calc(pp, path.getTgtSpeed(), pose, v.linearVel, dt);
        if (cmd != null) {
            RobotLog.ii(TAG, "@(%.1f,%.1f/%.1f) -> (%.1f,%.1f) power: (%.1f,%.1f,%.1f)",
                    pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()),
                    pp.x, pp.y,
                    cmd.axial, cmd.lateral, cmd.yaw);

            this.dt.update(cmd.axial, cmd.lateral, cmd.yaw);
        } else {
            RobotLog.ii(TAG, "done");
            done = true;
        }

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

    public MotionCommand calc(Vector2d pursuitPoint, double targetSpd, Pose2d pose, Vector2d linearVelocity,
                              double dt) {
        Vector2d vRel = FieldToRobot.rotateToRobotRel(pose.heading, linearVelocity);

        Vector2d pp = pursuitPoint;
        Vector2d ppRel = FieldToRobot.toRobotRel(pose, pp);

        double dist = ppRel.norm();
        double radialSpeed = vRel.norm();

        if (dist < POS_TOL && radialSpeed < VEL_TOL) {
            return null;
        }

        Vector2d ppNormRel = VectorCalc.normalize(ppRel);

        Vector2d v1 = ppNormRel.times(planSpeed(dt, dist, radialSpeed, targetSpd,
                Math.min(axMcs.accelLimit, latMcs.accelLimit),
                VEL_LIMIT));

        double axialPower = drivePower(dt, ppRel.x, vRel.x, v1.x, axMcs);
        double lateralPower = drivePower(dt, ppRel.y, vRel.y, v1.y, latMcs);

        Rotation2d r = getPursuitAngle(ppNormRel);

        double yaw = Math.toDegrees(r.toDouble()) * P_COEFF_ANG;

        return new MotionCommand(axialPower, lateralPower, 0);
    }

    private static double planSpeed(double dt, double dist, double v, double targetSpd, double accelLimit,
                                    double velLimit) {
        double a0 = -accelLimit;
        double vd = targetSpd;
        double z = vd * vd - 2 * a0 * dist;
        double v0;

        if (z > 0) {
            v0 = Math.sqrt(z);
        } else {
            // this should generally not happen
            v0 = velLimit;
        }

        double v1 = limitMagnitude(v0, velLimit);
        return v1;
    }

    private static double drivePower(double dt, double dist, double v, double v1, MotionControlSettings settings) {
        int s = (int) Math.signum(dist);
        double nextA = limitMagnitude((v1 - v) / dt * settings.pCoefficient,
                settings.accelLimit);
        double p = settings.ks * s + settings.kv * v + settings.ka * nextA;
        return p;
    }

    @Override
    public void cancel() {
        done = true;
    }
}
