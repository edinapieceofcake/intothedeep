package edu.edina.Libraries.Robot;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import edu.edina.Libraries.MotionControl.ILinearMechanism;
import edu.edina.Libraries.LinearMotion.LinearMechanismSettings;
import edu.edina.Libraries.LinearMotion.Units;
import kotlin.NotImplementedError;

public class ArmLift implements ILinearMechanism {
    private HardwareMap hardwareMap;
    private DcMotorEx left, right;
    private final VoltageSensor vs;
    private double nominalVolt;

    // 14.75 / 1710
    private double ppi = 1679.0 / 14.835;
    private double inchesPerP = 1.0 / ppi;
    private double avgPos = 0.0;

    private static final LinearMechanismSettings settings = new LinearMechanismSettings(
            "arm",
            Units.INCHES,
            0,
            0,
            0,
            0.5
    );

    public ArmLift(RobotHardware hw) {
        this.hardwareMap = hardwareMap;
        //left = hw.liftMotorLeft;
        //right = hw.liftMotorRight;
        vs = hw.voltageSensor;

        nominalVolt = 11.0;
    }

    @Override
    public void setPower(double power) {
        //double adjPower = power * nominalVolt / vs.getVoltage();
        //left.setPower(adjPower);
        //right.setPower(adjPower);

        left.setPower(power);
        right.setPower(power);
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
    public DualNum<Time> getPositionAndVelocity(boolean raw) {
        throw new NotImplementedError();
    }

    @Override
    public LinearMechanismSettings getSettings() {
        return settings;
    }
}