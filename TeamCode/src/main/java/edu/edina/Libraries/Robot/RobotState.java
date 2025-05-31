package edu.edina.Libraries.Robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotState {
    private VoltageSensor vs;
    private DcMotorEx extensionMotor, armMotor, leftMotor, rightMotor;

    private int extensionPos, armPos, leftPos, rightPos;
    private double voltage;

    public static double LIFT_MULT = -14.4 / 1615.0;
    public static double POS_AT_180_DEG_ARM = 4060;
    public static double EXTENSION_MULT = -11.0 / 1285.0;

    public RobotState(HardwareMap hw) {
        extensionMotor = hw.get(DcMotorEx.class, "extension_motor");
        armMotor = hw.get(DcMotorEx.class, "arm_motor");
        leftMotor = hw.get(DcMotorEx.class, "left_lift_motor");
        rightMotor = hw.get(DcMotorEx.class, "right_lift_motor");
        vs = hw.voltageSensor.iterator().next();
    }

    public void update(Telemetry telemetry) {
        extensionPos = extensionMotor.getCurrentPosition();
        armPos = armMotor.getCurrentPosition();
        leftPos = leftMotor.getCurrentPosition();
        rightPos = rightMotor.getCurrentPosition();
        voltage = vs.getVoltage();
    }

    public double getRightLiftPos() {
        return rightPos * LIFT_MULT;
    }

    public double getLeftLiftPos() {
        return leftPos * LIFT_MULT;
    }

    public double getLiftPos() {
        return (getLeftLiftPos() - getRightLiftPos()) / 2.0;
    }

    public double getArmPos() {
        return armPos * (180.0 / POS_AT_180_DEG_ARM);
    }

    public double getExtensionPos() {
        return extensionPos * EXTENSION_MULT;
    }

    public double getVoltage() {
        return voltage;
    }
}