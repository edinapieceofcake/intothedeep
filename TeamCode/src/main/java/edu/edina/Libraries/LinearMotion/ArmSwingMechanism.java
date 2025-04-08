package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import edu.edina.Libraries.Robot.Arm;
import edu.edina.Libraries.Robot.DrivingRobotHardware;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.Speedometer;

@Config
public class ArmSwingMechanism implements ILinearMechanism {
    private final Speedometer speedometer;
    private final VoltageSensor vs;
    private final DcMotorEx arm;
    private double zeroDegPos;

    private final double DEG_MULT = 180.0 / (705.0 - 4726.0);

    public static double KS = 5.8081e-2;
    public static double KV = 1.7329e-3;
    public static double KA = 2.6336e-4;
    public static double NOMINAL_ACCEL = 500;
    public static double STOP_ACCEL_MULT = 0.3;
    public static double STOP_T_TOL = 0.45;
    public static double STOP_X_TOL = 2;
    public static double MAX_JERK = 1e9;

    public static LinearMechanismSettings getStaticSettings() {
        return new LinearMechanismSettings(
                "arm swing", Units.DEGREES_NOT_WRAPPED,
                KS, KV, KA, 20,
                NOMINAL_ACCEL,
                STOP_ACCEL_MULT,
                STOP_T_TOL,
                STOP_X_TOL,
                MAX_JERK);
    }

    public ArmSwingMechanism(VoltageSensor vs, DcMotorEx arm) {
        speedometer = new Speedometer(3);
        this.vs = vs;
        this.arm = arm;
        zeroDegPos = -Arm.INITIAL_DEGREES_BELOW_HORIZONTAL / DEG_MULT;
    }

    @Override
    public void setPower(double power) {
        double actualPower = -power * 12 / vs.getVoltage();
        arm.setPower(actualPower);
    }

    @Override
    public double getPosition(boolean raw) {
        return getPositionAndVelocity(raw).get(0);
    }

    @Override
    public DualNum<Time> getPositionAndVelocity(boolean raw) {
        double position = arm.getCurrentPosition() + zeroDegPos;
        if (!raw)
            position *= DEG_MULT;

        speedometer.sample(position);
        return new DualNum<>(new double[]{position, speedometer.getSpeed()});
    }

    @Override
    public LinearMechanismSettings getSettings() {
        return getStaticSettings();
    }
}
