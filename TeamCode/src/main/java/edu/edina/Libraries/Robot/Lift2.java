package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import edu.edina.Libraries.Actions.MotionControlAction;
import edu.edina.Libraries.Actions.PidAction;
import edu.edina.Libraries.Actions.PidSettings;
import edu.edina.Libraries.LinearMotion.LinearMechanismSettings;
import edu.edina.Libraries.LinearMotion.Units;
import edu.edina.Libraries.MotionControl.IMotionControlLinearMechanism;

@Config
public class Lift2 {
    public static double KS = 6.1402e-2;
    public static double KV = 3.3710e-2;
    public static double KA = 2.8125e-2;

    public static double VEL_LIMIT = 200;
    public static double MAX_POWER = 0.6;
    public static double POS_TOLERANCE = 0.5;
    public static double VEL_TOLERANCE = 1;
    public static double POS_MULT = -14.4 / 1615.0;

    public static double P = .2;
    public static double I = 0;
    public static double D = 0;

    private Mechanism mechanism;

    public Lift2(Mechanism mechanism) {
        this.mechanism = mechanism;
    }

    public Action moveLift(double target) {
        PidSettings p = new PidSettings(P, I, D);

        return new SequentialAction(
                new MotionControlAction(target, 0, mechanism),
                new PidAction(target, p, mechanism)
        );
    }

    public static class Mechanism implements IMotionControlLinearMechanism {
        private final DcMotorEx motorLeft, motorRight;
        private final Speedometer speedometer;

        public Mechanism(HardwareMap hardwareMap) {
            speedometer = new Speedometer(3);

            motorRight = hardwareMap.get(DcMotorEx.class, "right_lift_motor");
            motorLeft = hardwareMap.get(DcMotorEx.class, "left_lift_motor");
            motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
            motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        @Override
        public void setPower(double power) {
            motorRight.setPower(power);
            motorLeft.setPower(power);
        }

        @Override
        public double getPosition(boolean raw) {
            return getPositionAndVelocity(raw).value();
        }

        @Override
        public DualNum<Time> getPositionAndVelocity(boolean raw) {
            double pos = (motorRight.getCurrentPosition() - motorLeft.getCurrentPosition()) / 2.0;
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
            return new LinearMechanismSettings("lift", Units.INCHES, KS, KV, KA, 5);
        }

        @Override
        public double getAccelCalDistance() {
            return 5;
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