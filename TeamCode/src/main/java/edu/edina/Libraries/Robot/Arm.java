package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import edu.edina.Libraries.Actions.ConstantPowerAction;
import edu.edina.Libraries.Actions.ControllingAction;
import edu.edina.Libraries.Actions.ControllingActionManager;
import edu.edina.Libraries.Actions.MotionControlAction;
import edu.edina.Libraries.Actions.PidAction;
import edu.edina.Libraries.Actions.PidSettings;
import edu.edina.Libraries.Actions.SinglePowerPid;
import edu.edina.Libraries.LinearMotion.IFeedForward;
import edu.edina.Libraries.LinearMotion.VoltageCompensation;
import edu.edina.Libraries.MotionControl.ICancelableAction;
import edu.edina.Libraries.MotionControl.IMotionControlLinearMechanism;
import edu.edina.Libraries.LinearMotion.LinearMechanismSettings;
import edu.edina.Libraries.LinearMotion.Units;

@Config
public class Arm {
    // positions
    public static double POS_SPECIMEN = 200;
    public static double POS_LOW_SPECIMEN = 215;
    public static double POS_SUBMERSIBLE = 193;
    public static double POS_HIGH_BASKET = 100;
    public static double POS_LOW_BASKET = 85;
    public static double POS_ARM_VERTICAL = 125;
    public static double POS_ARM_SCORE_BASKET_MIN = 85;
    //    public static double POS_ARM_SCORE_BASKET_MAX = 115;
    public static double POS_ARM_WALL = 33;
    public static double POS_GROUND = 205;
    public static double POS_GROUND_FRONT = 10;

    public static double POS_AT_180_DEG_ARM = 4060;
    public static double KS = 1.18e-1;
    public static double KV = 1.19e-3;
    public static double KA = 3e-4;

    // for linear motion
    public static double MOT_VEL_LIMIT = 300;
    public static double MOT_MAX_POWER = 0.65;
    public static double MOT_POS_TOLERANCE = 15;
    public static double MOT_VEL_TOLERANCE = 20;
    public static double MOT_VEL_COEF = 0.45;

    // for pid action
    public static double HOLD_P = 0.00001;
    public static double HOLD_I = 0.025;
    public static double HOLD_D = 0.001;

    public static double INTAKE_POWER = 0.3;
    public static double MAX_FEED_FWD_MULT = 0.0025;

    public static double CONST_POWER = 0.15;
    public static double POS_TOLERANCE = 2;

    private RobotState rS;

    private final Mechanism mechanism;
    private final ControllingActionManager conActMgr;
    private final VoltageCompensation vc;

    public Arm(RobotState rS, HardwareMap hw) {
        this.rS = rS;
        mechanism = new Arm.Mechanism(rS, hw);
        conActMgr = new ControllingActionManager();
        vc = new VoltageCompensation(rS);
    }

    public Action moveAndHold(double target) {
        return new ControllingAction(
                new SequentialAction(
                        new MotionControlAction(target, mechanism, vc, makeFeedFwd()),
                        new SinglePowerPid(CONST_POWER, target, POS_TOLERANCE, makeFeedFwd(), mechanism, vc)
                ),
                conActMgr);
    }

    public Action release() {
        return new InstantAction(() -> {
            mechanism.setPower(0);
            conActMgr.cancelControllingAction();
        });
    }

    public Action constantPower(double nominalPower) {
        return new ControllingAction(
                new ConstantPowerAction(nominalPower, mechanism, vc, null),
                conActMgr);
    }

    public Action moveArmWithPid(double target) {
        return new ControllingAction(
                new PidAction(target, getPidSettings(), mechanism, vc, makeFeedFwd()),
                conActMgr);
    }

    public Action holdPos() {
        return moveArmWithPid(rS.getExtensionPos());
    }

    public boolean at(double pos, double tol) {
        return Math.abs(rS.getArmPos() - pos) < tol;
    }

    private PidSettings getPidSettings() {
        return new PidSettings(HOLD_P, HOLD_I, HOLD_D);
    }

    private IFeedForward makeFeedFwd() {
        return new ArmFeedForward(rS);
    }

    public String getName() {
        return mechanism.getName();
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
                    MOT_VEL_COEF, 1);
        }
    }
}
