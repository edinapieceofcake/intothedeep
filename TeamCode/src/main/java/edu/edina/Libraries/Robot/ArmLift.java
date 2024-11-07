package edu.edina.Libraries.Robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class ArmLift implements ILinearMechanism {
    private HardwareMap hardwareMap;
    private final DcMotorEx left, right;
    private final VoltageSensor vs;
    private double nominalVolt;

    // 14.75 / 1710
    private double ppi = 1679.0 / 14.835;
    private double inchesPerP = 1.0 / ppi;
    private double avgPos = 0.0;

    private static final LinearMechanismSettings settings = new LinearMechanismSettings(
            0,
            0,
            0,
            0.5
    );

    public ArmLift(RobotHardware hw) {
        this.hardwareMap = hardwareMap;
        left = hw.liftMotorLeft;
        right = hw.liftMotorRight;
        vs = hw.voltageSensor;

        nominalVolt = 11.0;
    }

    @Override
    public void setPower(double power) {
        double adjPower = power * nominalVolt / vs.getVoltage();

        left.setPower(adjPower);
        right.setPower(adjPower);
    }

    @Override
    public double getPosition(boolean raw) {
        avgPos = (left.getCurrentPosition() + right.getCurrentPosition()) / 2.0;

        if (raw) {
            return avgPos;
        } else {
            return avgPos * inchesPerP;
        }
    }

    @Override
    public LinearMechanismSettings getSettings() {
        return settings;
    }
}