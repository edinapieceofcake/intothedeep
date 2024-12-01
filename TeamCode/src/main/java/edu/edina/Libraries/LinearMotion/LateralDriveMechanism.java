package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import edu.edina.Libraries.RoadRunner.ThreeDeadWheelLocalizer;
import edu.edina.Libraries.Robot.Drivetrain;
import edu.edina.Libraries.Robot.RobotHardware;

@Config
public class LateralDriveMechanism implements ILinearMechanism {
    private ThreeDeadWheelLocalizer odometry;
    private Drivetrain drivetrain;
    private VoltageSensor vs;
    private Pose2d pose;

    public static double KS = 0.066229;
    public static double KV = 0.015638;
    public static double KA = 0.0025;
    public static double NOMINAL_ACCEL = 1 / (2 * 3.4678e-3);
    public static double STOP_ACCEL_MULT = 0.3;
    public static double STOP_T_TOL = 0.45;
    public static double STOP_X_TOL = 0.3;
    public static double MAX_JERK = 60000;

    public LinearMechanismSettings settings = new LinearMechanismSettings(
            "axial drive", "inches",
            KS, KV, KA, 20,
            NOMINAL_ACCEL,
            STOP_ACCEL_MULT,
            STOP_T_TOL,
            STOP_X_TOL,
            MAX_JERK);

    public LateralDriveMechanism(RobotHardware hw) {
        drivetrain = hw.drivetrain;
        odometry = hw.odometry;
        pose = new Pose2d(new Vector2d(0, 0), 0);

        vs = hw.voltageSensor;
    }

    @Override
    public void setPower(double power) {
        double actualPower = power * 12 / vs.getVoltage();
        drivetrain.update(0, actualPower, 0);
    }

    @Override
    public double getPosition(boolean raw) {
        Twist2dDual<Time> t = odometry.update();
        pose = pose.plus(t.value());
        return pose.position.y;
    }

    @Override
    public DualNum<Time> getPositionAndVelocity(boolean raw) {
        Twist2dDual<Time> t = odometry.update();
        pose = pose.plus(t.value());
        double v = t.velocity().linearVel.y.value();
        return new DualNum<Time>(new double[]{pose.position.y, v});
    }

    @Override
    public LinearMechanismSettings getSettings() {
        return settings;
    }
}
