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
import edu.edina.Libraries.MotionControl.ICancelableAction;
import edu.edina.Libraries.MotionControl.IMotionControlLinearMechanism;
import edu.edina.Libraries.LinearMotion.LinearMechanismSettings;
import edu.edina.Libraries.LinearMotion.Units;

@Config
public class Arm2 {
    // positions
    public static double POS_SPECIMEN = 200;
    public static double POS_HIGH_BASKET = 120;
    public static double POS_LOW_BASKET = 100;

    public static double POS_AT_180_DEG_ARM = 4060;

    public static double KS = 1.18e-1;
    public static double KV = 1.19e-3;
    public static double KA = 3e-4;

    // for linear motion
    public static double MOT_VEL_LIMIT = 200;
    public static double MOT_MAX_POWER = 0.6;
    public static double MOT_POS_TOLERANCE = 15;
    public static double MOT_VEL_TOLERANCE = 20;
    public static double MOT_VEL_COEF = 0.7;

    // for pid action
    public static double HOLD_P = 0.01;
    public static double HOLD_I = 0.05;
    public static double HOLD_D = 0.002;
    private RobotState rS;

    private Mechanism mechanism;

    public Arm2(RobotState rS, HardwareMap hw) {
        this.rS = rS;
        mechanism = new Arm2.Mechanism(rS, hw);

    }

    public Action moveArm(double target) {
        return new SequentialAction(
                new MotionControlAction(target, mechanism),
                new PidAction(target, getPidSettings(), mechanism)
        );
    }

    public Action moveArmWithPid(double target) {
        return new PidAction(target, getPidSettings(), mechanism);
    }

    public Action holdPos() {
        return new PidAction(rS.getExtensionPos(), getPidSettings(), mechanism);
    }

    private PidSettings getPidSettings() {
        return new PidSettings(HOLD_P, HOLD_I, HOLD_D);
    }

    public static class Mechanism implements IMotionControlLinearMechanism {
        private final DcMotorEx motor;
        private ICancelableAction currentAction;
        private final RobotState robotState;

        public Mechanism(RobotState rS, HardwareMap hw) {
            robotState = rS;

            motor = hw.get(DcMotorEx.class, "arm_motor");
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        @Override
        public String getName() {
            return "arm";
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
            double pos = robotState.getArmPos();
            double vel = robotState.getArmSpeed();

            DualNum<Time> posVel = new DualNum<>(new double[]{pos, vel});

            return posVel;
        }

        @Override
        public LinearMechanismSettings getSettings() {
            return new LinearMechanismSettings(getName(), Units.DEGREES, KS, KV, KA, 40);
        }

        @Override
        public double getAccelCalDistance() {
            return 40;
        }

        @Override
        public MotionControlSettings getMotionSettings() {
            return new MotionControlSettings(KS, KV, KA,
                    MOT_VEL_LIMIT, MOT_MAX_POWER,
                    MOT_POS_TOLERANCE, MOT_VEL_TOLERANCE,
                    MOT_VEL_COEF);
        }

        @Override
        public void setCurrentAction(ICancelableAction action) {
            if (currentAction != null)
                currentAction.cancel();

            currentAction = action;
        }
    }
}
