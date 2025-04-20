package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import edu.edina.Libraries.LinearMotion.ILinearMechanism;
import edu.edina.Libraries.LinearMotion.LinearMechanismSettings;
import edu.edina.Libraries.LinearMotion.Units;

@Config
public class ArmMech implements ILinearMechanism {
    public static double KS = 0;
    public static double KV = 0;
    public static double KA = 0;
    public static double POS_AT_180_DEG = 0;

    private final double posMult;
    private final DcMotorEx motor;
    private final Speedometer speedometer;

    public ArmMech(HardwareMap hw) {
        posMult = 180.0 / POS_AT_180_DEG;

        speedometer = new Speedometer(3);

        motor = hw.get(DcMotorEx.class, "extension_motor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void setPower(double power) {
        motor.setPower(power);
    }

    @Override
    public double getPosition(boolean raw) {
        return getPositionAndVelocity(raw).value();
    }

    @Override
    public DualNum<Time> getPositionAndVelocity(boolean raw) {
        double pos = motor.getCurrentPosition();
        speedometer.sample(pos);
        double vel = speedometer.getSpeed();

        DualNum<Time> posVel = new DualNum<>(new double[]{pos, vel});

        if (raw)
            return posVel;
        else
            return posVel.times(posMult);
    }

    @Override
    public LinearMechanismSettings getSettings() {
        return new LinearMechanismSettings("arm", Units.DEGREES, KS, KV, KA, 0);
    }
}
