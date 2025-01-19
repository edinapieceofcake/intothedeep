package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import edu.edina.Libraries.Robot.Speedometer;

@Config
public class TestMechanism implements ILinearMechanism {
    private DcMotorEx motor;
    private VoltageSensor vs;
    private Speedometer speedometer;

    public static double DEG_MULT = 0;

    public static double KS = 0.01;
    public static double KV = 0.01;
    public static double KA = 0.01;
    public static double NOMINAL_ACCEL = 1 / (2 * KA);
    public static double STOP_ACCEL_MULT = 0.3;
    public static double STOP_T_TOL = 0.45;
    public static double STOP_X_TOL = 0.3;
    public static double MAX_JERK = 6000;

    public static LinearMechanismSettings getStaticSettings() {
        return new LinearMechanismSettings(
                "test motor", Units.DEGREES,
                KS, KV, KA, 180,
                NOMINAL_ACCEL,
                STOP_ACCEL_MULT,
                STOP_T_TOL,
                STOP_X_TOL,
                MAX_JERK);
    }

    public TestMechanism(HardwareMap hw) {
        speedometer = new Speedometer(3);
        motor = hw.get(DcMotorEx.class, "test");
        vs = hw.voltageSensor.iterator().next();
    }

    @Override
    public void setPower(double power) {
        double actualPower = power * 12 / vs.getVoltage();
        motor.setPower(actualPower);
    }

    @Override
    public double getPosition(boolean raw) {
        return getPositionAndVelocity(raw).get(0);
    }

    @Override
    public DualNum<Time> getPositionAndVelocity(boolean raw) {
        double position = motor.getCurrentPosition();
        if (!raw)
            position *= DEG_MULT;

        speedometer.sample(position);
        return new DualNum<>(new double[] {position, speedometer.getSpeed()});
    }

    @Override
    public LinearMechanismSettings getSettings() {
        return getStaticSettings();
    }
}
