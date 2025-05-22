package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;

import edu.edina.Libraries.Actions.FeedForward;
import edu.edina.Libraries.MotionControl.IMotionControlLinearMechanism;
import edu.edina.Libraries.Robot.MotionControlSettings;

public class FeedForwardMechanism implements IMotionControlLinearMechanism {
    private final IMotionControlLinearMechanism linearMechanism;
    private final FeedForward feedFwd;

    private DualNum<Time> lastPosAndVel;

    public FeedForwardMechanism(IMotionControlLinearMechanism linearMechanism, FeedForward feedFwd) {
        this.linearMechanism = linearMechanism;
        this.feedFwd = feedFwd;
        lastPosAndVel = new DualNum<>(new double[]{0, 0});
    }

    @Override
    public MotionControlSettings getMotionSettings() {
        return linearMechanism.getMotionSettings();
    }

    @Override
    public void setPower(double power) {
        double x = lastPosAndVel.value();
        double ff = feedFwd.getFeedForwardPower(x);
        linearMechanism.setPower(power + ff);
    }

    @Override
    public double getPosition(boolean raw) {
        return getPositionAndVelocity(raw).value();
    }

    @Override
    public DualNum<Time> getPositionAndVelocity(boolean raw) {
        lastPosAndVel = linearMechanism.getPositionAndVelocity(raw);
        return lastPosAndVel;
    }

    @Override
    public LinearMechanismSettings getSettings() {
        return linearMechanism.getSettings();
    }
}
