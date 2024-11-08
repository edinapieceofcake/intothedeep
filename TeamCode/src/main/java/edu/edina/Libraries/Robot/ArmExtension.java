package edu.edina.Libraries.Robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ArmExtension implements ILinearMechanism {
    private static final double inchesPerVolt = 6.375 / 4.4;
    private static final LinearMechanismSettings settings = new LinearMechanismSettings(
            0.0552,
            0.0127,
            0,
            0.5
    );

    private final CRServo servo;
    private final AnalogInput encoder;
    private double offset, lastVolts;

    public ArmExtension(RobotHardware hw) {
        servo = hw.slideServo;
        encoder = hw.slideEncoder;

        lastVolts = encoder.getVoltage();
        offset = -lastVolts;
    }

    @Override
    public void setPower(double power) {
        servo.setPower(-power);
    }

    @Override
    public double getPosition(boolean raw) {
        double v = encoder.getVoltage();

        double maxV = encoder.getMaxVoltage();

        double lo = maxV / 3;
        double hi = maxV - lo;

        if (v < lo && lastVolts > hi)
            offset += maxV;
        else if (v > hi && lastVolts < lo)
            offset -= maxV;

        lastVolts = v;

        double pos = v + offset;
        if (raw)
            return pos;
        else
            return inchesPerVolt * pos;
    }

    @Override
    public LinearMechanismSettings getSettings() {
        return settings;
    }
}