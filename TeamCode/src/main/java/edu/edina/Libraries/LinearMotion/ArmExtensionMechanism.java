package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import edu.edina.Libraries.MotionControl.ILinearMechanism;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.Speedometer;

@Config
public class ArmExtensionMechanism implements ILinearMechanism {
    private final Speedometer speedometer;
    private final VoltageSensor vs;
    private final DcMotorEx extension;

    private final double INCH_MULT = 5.75 / -659;

    public static double KS = 3.8643e-3;
    public static double KV = 4.3581e-2;
    public static double KA = 4.6867e-3;
    public static double NOMINAL_ACCEL = 1 / (2 * KA);
    public static double STOP_ACCEL_MULT = 0.3;
    public static double STOP_T_TOL = 0.45;
    public static double STOP_X_TOL = 0.3;
    public static double MAX_JERK = 6000;

    public static LinearMechanismSettings getStaticSettings() {
        return new LinearMechanismSettings(
                "axial drive", Units.INCHES,
                KS, KV, KA, 12,
                NOMINAL_ACCEL,
                STOP_ACCEL_MULT,
                STOP_T_TOL,
                STOP_X_TOL,
                MAX_JERK);
    }

    public ArmExtensionMechanism(RobotHardware hw) {
        speedometer = new Speedometer(3);
        vs = hw.getVoltageSensor();
        extension = hw.getExtension();
    }

    @Override
    public void setPower(double power) {
        double actualPower = -power * 12 / vs.getVoltage();
        extension.setPower(actualPower);
    }

    @Override
    public double getPosition(boolean raw) {
        return getPositionAndVelocity(raw).get(0);
    }

    @Override
    public DualNum<Time> getPositionAndVelocity(boolean raw) {
        double position = extension.getCurrentPosition();
        if (!raw)
            position *= INCH_MULT;

        speedometer.sample(position);
        return new DualNum<>(new double[]{position, speedometer.getSpeed()});
    }

    @Override
    public LinearMechanismSettings getSettings() {
        return getStaticSettings();
    }
}