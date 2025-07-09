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

import edu.edina.Libraries.Actions.ControllingAction;
import edu.edina.Libraries.Actions.ControllingActionManager;
import edu.edina.Libraries.Actions.MotionControlAction;
import edu.edina.Libraries.Actions.PidAction;
import edu.edina.Libraries.Actions.PidSettings;
import edu.edina.Libraries.LinearMotion.LinearMechanismSettings;
import edu.edina.Libraries.LinearMotion.Units;
import edu.edina.Libraries.LinearMotion.VoltageCompensation;
import edu.edina.Libraries.MotionControl.ICancelableAction;
import edu.edina.Libraries.MotionControl.IMotionControlLinearMechanism;

@Config
public class Lift {
    public static double LIFT_MULT = 14.4 / 1615.0;

    public static double POS_HIGH_BASKET = 15;
    public static double POS_LOW_BASKET = 12;
    public static double POS_BOTTOM = 0;

    public static double KS = 6.1402e-2;
    public static double KV = 3.3710e-2;
    public static double KA = 2.8125e-2;

    public static double MOT_VEL_LIMIT = 200;
    public static double MOT_MAX_POWER = 0.6;
    public static double MOT_POS_TOLERANCE = 0.5;
    public static double MOT_VEL_TOLERANCE = 1;
    public static double MOT_VEL_COEF = .2;

    public static double HOLD_P = 0.4;
    public static double HOLD_I = 0.04;
    public static double HOLD_D = 0;

    private final Mechanism mechanism;
    private final ControllingActionManager conActMgr;
    private final VoltageCompensation vc;

    public Lift(RobotState rS, HardwareMap hw) {
        mechanism = new Lift.Mechanism(rS, hw);
        conActMgr = new ControllingActionManager();
        vc = new VoltageCompensation(rS);
    }

    public Action moveLift(double target) {
        return new ControllingAction(
                new SequentialAction(
                        new MotionControlAction(target, mechanism, vc, null),
                        new PidAction(target, getPidSettings(), mechanism, vc, null)
                ),
                conActMgr);
    }

    private PidSettings getPidSettings() {
        return new PidSettings(HOLD_P, HOLD_I, HOLD_D);
    }

    public static class Mechanism implements IMotionControlLinearMechanism {
        private final DcMotorEx motorLeft, motorRight;
        private ICancelableAction currentAction;
        private final RobotState robotState;

        public Mechanism(RobotState rS, HardwareMap hw) {
            robotState = rS;

            motorRight = hw.get(DcMotorEx.class, "right_lift_motor");
            motorLeft = hw.get(DcMotorEx.class, "left_lift_motor");
            motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
            motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        @Override
        public String getName() {
            return "lift";
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
            double pos = robotState.getLiftPos();
            double vel = robotState.getLiftSpeed();

            DualNum<Time> posVel = new DualNum<>(new double[]{pos, vel});

            return posVel;
        }

        @Override
        public LinearMechanismSettings getSettings() {
            return new LinearMechanismSettings(getName(), Units.INCHES, KS, KV, KA, 5);
        }

        @Override
        public double getAccelCalDistance() {
            return 5;
        }

        @Override
        public MotionControlSettings getMotionSettings() {
            return new MotionControlSettings(KS, KV, KA,
                    MOT_VEL_LIMIT, MOT_MAX_POWER,
                    MOT_POS_TOLERANCE, MOT_VEL_TOLERANCE,
                    MOT_VEL_COEF);
        }
    }
}