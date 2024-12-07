package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import edu.edina.Libraries.RoadRunner.Localizer;
import edu.edina.Libraries.RoadRunner.ThreeDeadWheelLocalizer;
import edu.edina.Libraries.Robot.Drivetrain;
import edu.edina.Libraries.Robot.DrivingRobotHardware;
import edu.edina.Libraries.Robot.Odometry;
import edu.edina.Libraries.Robot.RobotHardware;

public class RotationalDriveMechanism implements ILinearMechanism {
    private final Odometry odometry;
    private final Drivetrain drivetrain;
    private final VoltageSensor vs;

    public static double KS = 5.6038e-2;
    public static double KV = 4.3259e-3;
    public static double KA = 0.001;
    public static double NOMINAL_ACCEL = 50;
    public static double STOP_ACCEL_MULT = 0.3;
    public static double STOP_T_TOL = 0.45;
    public static double STOP_X_TOL = 2;
    public static double MAX_JERK = 6000;

    public static LinearMechanismSettings getStaticSettings() {
        return new LinearMechanismSettings(
                "axial drive", Units.DEGREES,
                KS, KV, KA, 20,
                NOMINAL_ACCEL,
                STOP_ACCEL_MULT,
                STOP_T_TOL,
                STOP_X_TOL,
                MAX_JERK);
    }

    public RotationalDriveMechanism(DrivingRobotHardware hw) {
        drivetrain = hw.getDrivetrain();
        odometry = hw.getOdometry();
        vs = hw.getVoltageSensor();
    }

    @Override
    public void setPower(double power) {
        double actualPower = power * 12 / vs.getVoltage();
        drivetrain.update(0, 0, -actualPower);
    }

    @Override
    public double getPosition(boolean raw) {
        return getPositionAndVelocity(raw).value();
    }

    @Override
    public DualNum<Time> getPositionAndVelocity(boolean raw) {
        odometry.update();
        Pose2d p = odometry.getPoseEstimate();
        PoseVelocity2d v = odometry.getVelocityEstimate();
        return new DualNum<>(new double[]{Math.toDegrees(p.heading.toDouble()), Math.toDegrees(v.angVel)});
    }

    @Override
    public LinearMechanismSettings getSettings() {
        return getStaticSettings();
    }
}
