package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;

import edu.edina.Libraries.MotionControl.ICancelableAction;
import edu.edina.Libraries.MotionControl.IMotionControlLinearMechanism;
import edu.edina.Libraries.Robot.MotionControlSettings;
import edu.edina.Libraries.Robot.RobotState;

public class VoltageCompensatingMechanism implements IMotionControlLinearMechanism {
    private final IMotionControlLinearMechanism mechanism;
    private final RobotState robotState;

    public VoltageCompensatingMechanism(IMotionControlLinearMechanism mechanism, RobotState robotState) {
        this.mechanism = mechanism;
        this.robotState = robotState;
    }

    @Override
    public String getName() {
        return String.format("%s-vs", mechanism.getName());
    }

    @Override
    public double getAccelCalDistance() {
        return mechanism.getAccelCalDistance();
    }

    @Override
    public MotionControlSettings getMotionSettings() {
        return mechanism.getMotionSettings();
    }

    @Override
    public void setCurrentAction(ICancelableAction action) {
        mechanism.setCurrentAction(action);
    }

    @Override
    public void setPower(double power) {
        double v = robotState.getAverageVoltage();
        double multiplier = 12 / v;
        mechanism.setPower(power * multiplier);
    }

    @Override
    public double getPosition(boolean raw) {
        return mechanism.getPosition(raw);
    }

    @Override
    public DualNum<Time> getPositionAndVelocity(boolean raw) {
        return mechanism.getPositionAndVelocity(raw);
    }

    @Override
    public LinearMechanismSettings getSettings() {
        return mechanism.getSettings();
    }
}
