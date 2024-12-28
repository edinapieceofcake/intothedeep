package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.dashboard.config.Config;
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

@Config
public class LateralDriveMechanism implements ILinearMechanism {
    private final Odometry odometry;
    private final Drivetrain drivetrain;
    private final VoltageSensor vs;

    /* real robot
    public static double KS = 0.12338;
    public static double KV = 0.017618;
    public static double KA = 0.0042431;
    public static double NOMINAL_ACCEL = 1 / (2 * 3.4678e-3);
    public static double STOP_ACCEL_MULT = 0.6;
    public static double STOP_T_TOL = 0.6;
    public static double STOP_X_TOL = 0.32;
    public static double MAX_JERK = 70000;
     */

    /* test robot*/
    public static double KS = 0.12338;
    public static double KV = 0.017618;
    public static double KA = 0.0042431;
    public static double NOMINAL_ACCEL = 1 / (2 * 3.4678e-3);
    public static double STOP_ACCEL_MULT = 0.6;
    public static double STOP_T_TOL = 0.6;
    public static double STOP_X_TOL = 0.32;
    public static double MAX_JERK = 7000;

    public static LinearMechanismSettings getStaticSettings() {
        return new LinearMechanismSettings(
                "lateral drive", Units.INCHES,
                KS, KV, KA, 20,
                NOMINAL_ACCEL,
                STOP_ACCEL_MULT,
                STOP_T_TOL,
                STOP_X_TOL,
                MAX_JERK);
    }

    public LateralDriveMechanism(DrivingRobotHardware hw) {
        drivetrain = hw.getDrivetrain();
        odometry = hw.getOdometry();
        vs = hw.getVoltageSensor();
    }

    @Override
    public void setPower(double power) {
        double actualPower = power * 12 / vs.getVoltage();
        drivetrain.update(0, -actualPower, 0);
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
        return new DualNum<>(new double[]{p.position.y, v.linearVel.y});
    }

    @Override
    public LinearMechanismSettings getSettings() {
        return getStaticSettings();
    }
}
