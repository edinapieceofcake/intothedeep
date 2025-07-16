package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import edu.edina.Libraries.Actions.ControllingAction;
import edu.edina.Libraries.Actions.ControllingActionManager;
import edu.edina.Libraries.Actions.MotionControlAction;
import edu.edina.Libraries.Actions.PidAction;
import edu.edina.Libraries.Actions.PidSettings;
import edu.edina.Libraries.LinearMotion.VoltageCompensation;
import edu.edina.Libraries.MotionControl.ICancelableAction;
import edu.edina.Libraries.MotionControl.IMotionControlLinearMechanism;
import edu.edina.Libraries.LinearMotion.LinearMechanismSettings;
import edu.edina.Libraries.LinearMotion.Units;

@Config
public class Extension {
    public static double POS_GROUND_INTAKE = 2;
    public static double POS_HIGH_BASKET = 13;
    public static double POS_LOW_BASKET = 8;
    public static double POS_CHAMBER = 3;

    public static double KS = 0.06;
    public static double KV = 3.4e-2;
    public static double KA = 0.03;

    public static double MOT_VEL_LIMIT = 25;
    public static double MOT_MAX_POWER = 1;
    public static double MOT_POS_TOLERANCE = 1;
    public static double MOT_VEL_TOLERANCE = 2;
    public static double MOT_VEL_COEF = 0.15;
    public static double HOLD_P = 0.0001;
    public static double HOLD_I = 0.03;
    public static double HOLD_D = 0.0;

    public static double EXTENSION_MULT = 0.00846740050804403;

    public static double EXTENSION_RETRACTED_INCHES = 1;
    public static double INIT_EXTENSION_SUB = 5;

    public static double MANUAL_ADJ_MULT = 0.12;

    private final Mechanism mechanism;
    private final RobotState rS;
    private final ControllingActionManager conActMgr;
    private final VoltageCompensation vc;

    public Extension(RobotState rS, HardwareMap hw) {
        this.rS = rS;
        mechanism = new Extension.Mechanism(rS, hw);
        conActMgr = new ControllingActionManager();
        vc = new VoltageCompensation(rS);
    }

    public Action moveAndHold(double target) {
        return new ControllingAction(
                new SequentialAction(
                        new MotionControlAction(target, mechanism, vc, null),
                        new PidAction(target, getPidSettings(), mechanism, vc, null)),
                conActMgr);
    }

    public boolean at(double pos, double tol) {
        return Math.abs(rS.getExtensionPos() - pos) < tol;
    }

//    public Action moveExtensionWithPid(double target) {
//        return new ControllingAction(new PidAction(target, getPidSettings(), mechanism, vc, null), conActMgr);
//    }

    public Action manuallyAdjust(double power) {
        conActMgr.cancelControllingAction();
        mechanism.setPower(power);

        double v = rS.getExtensionSpeed();
        double x = rS.getExtensionPos();
        double target = x + v * MANUAL_ADJ_MULT;

        return new ControllingAction(
                new SequentialAction(
                        new WaitForTime(10),
                        new InstantAction(() -> mechanism.setPower(0)),
                        new PidAction(target, getPidSettings(), mechanism, vc, null)),
                conActMgr
        );
    }

    public Action makeResetAction() {
        return new ControllingAction(
                new ResetExtensionAction(rS, mechanism),
                conActMgr);
    }

    public Action release() {
        return new InstantAction(() -> {
            mechanism.setPower(0);
            conActMgr.cancelControllingAction();
        });
    }

    private PidSettings getPidSettings() {
        return new PidSettings(HOLD_P, HOLD_I, HOLD_D);
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

            motor = hw.get(DcMotorEx.class, "extension_motor");
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        @Override
        public String getName() {
            return "extension";
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
            double pos = robotState.getExtensionPos();
            double vel = robotState.getExtensionSpeed();

            DualNum<Time> posVel = new DualNum<>(new double[]{pos, vel});

            return posVel;
        }

        @Override
        public LinearMechanismSettings getSettings() {
            return new LinearMechanismSettings(getName(), Units.INCHES, KS, KV, KA, 2);
        }

        @Override
        public double getAccelCalDistance() {
            return 2;
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
