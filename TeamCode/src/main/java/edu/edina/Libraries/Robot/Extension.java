package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import edu.edina.Libraries.MotionControl.IMotionControlLinearMechanism;
import edu.edina.Libraries.LinearMotion.LinearMechanismSettings;
import edu.edina.Libraries.LinearMotion.Units;

@Config
public class Extension {
    public static double KS = 1.18e-1;
    public static double KV = 1.19e-3;
    public static double KA = 3e-4;
    public static double POS_MULT = 0;

    public static double VEL_LIMIT = 1;
    public static double MAX_POWER = 1;
    public static double POS_TOLERANCE = 1;
    public static double VEL_TOLERANCE = 1;
    public static double P = 0.1;

    public static class Mechanism implements IMotionControlLinearMechanism {
        private final DcMotorEx motor;
        private final Speedometer speedometer;

        public Mechanism(HardwareMap hw) {
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
                return posVel.times(POS_MULT);
        }

        @Override
        public LinearMechanismSettings getSettings() {
            return new LinearMechanismSettings("extension", Units.INCHES, KS, KV, KA, 40);
        }

        @Override
        public MotionControlSettings getMotionSettings() {
            return new MotionControlSettings(KS, KV, KA,
                    VEL_LIMIT, MAX_POWER,
                    POS_TOLERANCE, VEL_TOLERANCE,
                    P);
        }
    }
}
