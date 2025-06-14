package edu.edina.Libraries.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import edu.edina.Libraries.Actions.MotionControlAction;
import edu.edina.Libraries.Actions.PidAction;
import edu.edina.Libraries.Actions.PidSettings;
import edu.edina.Libraries.MotionControl.ICancelableAction;
import edu.edina.Libraries.MotionControl.IMotionControlLinearMechanism;
import edu.edina.Libraries.LinearMotion.LinearMechanismSettings;
import edu.edina.Libraries.LinearMotion.Units;

@Config
public class Arm2 {
    public static double KS = 1.18e-1;
    public static double KV = 1.19e-3;
    public static double KA = 3e-4;

    // for linear motion
    public static double VEL_LIMIT = 200;
    public static double MAX_POWER = 0.6;
    public static double POS_TOLERANCE = 15;
    public static double VEL_TOLERANCE = 20;
    public static double VEL_COEF = 0.7;

    // for pid action
    public static double P = 0.005;
    public static double I = 0;
    public static double D = 0.000002;
    private RobotState rS;

    private PidSettings p;
    private Mechanism mechanism;

    public Arm2(RobotState rS, HardwareMap hw) {
        this.rS = rS;
        mechanism = new Arm2.Mechanism(rS, hw);
        p = new PidSettings(P, I, D);
    }

    public Action moveArm(double target) {
        return new SequentialAction(
                new MotionControlAction(target, mechanism),
                new PidAction(target, p, mechanism)
        );
    }

    public Action moveArmWithPid(double target) {
        return new PidAction(target, p, mechanism);
    }

    public Action holdPos() {
        return new PidAction(rS.getExtensionPos(), p, mechanism);
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
    }
}
