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
public class Extension {
    public static double KS = 0.06;
    public static double KV = 3.4e-2;
    public static double KA = 3e-3;

    public static double VEL_LIMIT = 1;
    public static double MAX_POWER = 1;
    public static double POS_TOLERANCE = 0.25;
    public static double VEL_TOLERANCE = 1;
    public static double VEL_COEF = 0;
    public static double P = 0;
    public static double I = 0;
    public static double D = 0;

    private PidSettings p;
    private Mechanism mechanism;
    private RobotState rS;

    public Extension(RobotState rS, HardwareMap hw) {
        this.rS = rS;
        mechanism = new Extension.Mechanism(rS, hw);
        p = new PidSettings(P, I, D);
    }

    public Action moveExtension(double target) {
        return new SequentialAction(
                new MotionControlAction(target, mechanism),
                new PidAction(target, p, mechanism)
        );
    }

    public Action moveExtensionWithPid(double target) {
        return new PidAction(target, p, mechanism);
    }

    public Action holdPos() {
        return new PidAction(rS.getExtensionPos(), p, mechanism);
    }

    public void setPower(double power) {
        mechanism.setPower(power);
    }

    public void cancelAction() {
        mechanism.setCurrentAction(null);
    }

    public boolean hasCurrentAction() {
        return mechanism.hasCurrentAction();
    }

    public static class Mechanism implements IMotionControlLinearMechanism {
        private final DcMotorEx motor;
        private ICancelableAction currentAction;
        private final RobotState robotState;

        public Mechanism(RobotState rS, HardwareMap hw) {
            robotState = rS;

            motor = hw.get(DcMotorEx.class, "extension_motor");
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
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
            return new LinearMechanismSettings("extension", Units.INCHES, KS, KV, KA, 2);
        }

        @Override
        public double getAccelCalDistance() {
            return 2;
        }

        @Override
        public MotionControlSettings getMotionSettings() {
            return new MotionControlSettings(KS, KV, KA,
                    VEL_LIMIT, MAX_POWER,
                    POS_TOLERANCE, VEL_TOLERANCE,
                    VEL_COEF);
        }

        @Override
        public void setCurrentAction(ICancelableAction action) {
            if (currentAction != null)
                currentAction.cancel();

            currentAction = action;
        }

        public boolean hasCurrentAction() {
            return currentAction != null;
        }
    }
}
