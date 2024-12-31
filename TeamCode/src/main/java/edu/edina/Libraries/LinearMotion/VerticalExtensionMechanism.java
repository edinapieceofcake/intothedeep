package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.Speedometer;

@Config
public class VerticalExtensionMechanism implements ILinearMechanism {
    private Speedometer speedometer;
    private VoltageSensor vs;
    private DcMotorEx left;
    private DcMotorEx right;

    private static final double INCH_MULT = 14 / 1589.0 ;

    public static double KS = 6.9467e-02;
    public static double KV = 3.8799e-02;
    public static double KA = 8.4763e-03;
    public static double NOMINAL_ACCEL = 1 / (2 * KA);
    public static double STOP_ACCEL_MULT = 0.7;
    public static double STOP_T_TOL = 0.45;
    public static double STOP_X_TOL = 0.5;
    public static double MAX_JERK = 7000;

    public static LinearMechanismSettings getStaticSettings() {
        return new LinearMechanismSettings(
                "vertical mechanism", Units.INCHES,
                KS, KV, KA, 14,
                NOMINAL_ACCEL,
                STOP_ACCEL_MULT,
                STOP_T_TOL,
                STOP_X_TOL,
                MAX_JERK);
    }

    public VerticalExtensionMechanism(RobotHardware robotHardware) {
        HardwareMap hw = robotHardware.getOpMode().hardwareMap;
        speedometer = new Speedometer(3);
        vs = robotHardware.getVoltageSensor();
        left = hw.get(DcMotorEx.class, "left_lift_motor");
        right = hw.get(DcMotorEx.class, "right_lift_motor");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void setPower(double power) {
        double actualPower = power * 12 / vs.getVoltage();
        left.setPower(actualPower);
        right.setPower(actualPower);
    }

    @Override
    public double getPosition(boolean raw) {
        return getPositionAndVelocity(raw).get(0);
    }

    @Override
    public DualNum<Time> getPositionAndVelocity(boolean raw) {
        double averagePos= (left.getCurrentPosition() - right.getCurrentPosition()) / 2.0;
        if (!raw)
            averagePos *= INCH_MULT;

        speedometer.sample(averagePos);
        return new DualNum<>(new double[]{averagePos, speedometer.getSpeed()});
    }

    @Override
    public LinearMechanismSettings getSettings() {
        return getStaticSettings();
    }
}
