package edu.edina.Libraries.PurePursuit;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import edu.edina.Libraries.Robot.Drivetrain;
import edu.edina.Libraries.Robot.FieldToRobot;
import edu.edina.Libraries.Robot.MotionControlSettings;
import edu.edina.Libraries.Robot.Odometry;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.VectorCalc;

public class PurePursuitAction implements Action {
    public static double MAX_POWER = 0.75;
    public static double POS_TOL = 1;
    public static double VEL_TOL = 1;
    public static double P_COEFF = 1;

    public static double AXIAL_KA = 0.0025;

    public static double AXIAL_KS = 0.066229;
    public static double AXIAL_KV = 0.015638;
    public static double LAT_KA = 0.0042431;
    public static double LAT_KS = 0.12338;
    public static double LAT_KV = 0.017618;

    private PurePursuit purePursuit;
    private double tgtSpeed, maxSpeed, radius;
    private Odometry odometry;
    private Drivetrain dt;
    private Vector2d vecKs, vecKv, vecKa;

    private boolean done;

    public PurePursuitAction(Vector2d[] path, double tgtSpeed, double maxSpeed, double radius, RobotHardware hw) {
        purePursuit = new PurePursuit(path, false);
        this.tgtSpeed = tgtSpeed;
        this.radius = radius;
        this.maxSpeed = maxSpeed;
        odometry = hw.getOdometry();
        dt = hw.getDrivetrain();
        done = false;

        vecKs = new Vector2d(AXIAL_KS, LAT_KS);
        vecKv = new Vector2d(AXIAL_KV, LAT_KV);
        vecKa = new Vector2d(AXIAL_KA, LAT_KA);
    }

    private double limitMagnitude(double x, double maxMag) {
        return Math.abs(x) < maxMag ? x : maxMag * Math.signum(x);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        odometry.update();
        Pose2d p = odometry.getPoseEstimate();
        PoseVelocity2d v = odometry.getVelocityEstimate();

        purePursuit.calcNextPursuitPoint(p.position, radius);
        Vector2d pp = purePursuit.getPursuitPoint();
        Vector2d ppRel = FieldToRobot.toRobotRel(p, pp);

        Vector2d ppNormRel = VectorCalc.normalize(ppRel);
        double ks = ppNormRel.dot(vecKs);
        double kv = ppNormRel.dot(vecKv);
        double ka = ppNormRel.dot(vecKa);

          boolean keepRunning = false;
        return keepRunning;
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
        double nextA = limitMagnitude((v1 - v) / dt / settings.pCoefficient, settings.accelLimit);
        double p = settings.ks * s + settings.kv * v + settings.ka * nextA;

        return p;
    }
}
