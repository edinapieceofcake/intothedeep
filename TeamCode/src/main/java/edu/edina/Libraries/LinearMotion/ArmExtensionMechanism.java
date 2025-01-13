package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import edu.edina.Libraries.Robot.DrivingRobotHardware;
import edu.edina.Libraries.Robot.Speedometer;

public class ArmExtensionMechanism implements ILinearMechanism{
    private final Speedometer speedometer;
    private final VoltageSensor vs;
    private final DcMotorEx extension;

    private final double INCH_MULT = 0;

    public static double KS = 0.1;
    public static double KV = 0.01;
    public static double KA = 0.001;
    public static double NOMINAL_ACCEL = 1 / (2 * KA);
    public static double STOP_ACCEL_MULT = 0.3;
    public static double STOP_T_TOL = 0.45;
    public static double STOP_X_TOL = 0.3;
    public static double MAX_JERK = 6000;

    public static LinearMechanismSettings getStaticSettings() {
        return new LinearMechanismSettings(
                "axial drive", Units.INCHES,
                KS, KV, KA, 20,
                NOMINAL_ACCEL,
                STOP_ACCEL_MULT,
                STOP_T_TOL,
                STOP_X_TOL,
                MAX_JERK);
    }

    public ArmExtensionMechanism(DrivingRobotHardware hw) {
        speedometer = new Speedometer(3);
        vs = hw.getVoltageSensor();
        extension = hw.getExtension();
    }

    @Override
    public void setPower(double power) {
        double actualPower = power * 12 / vs.getVoltage();
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