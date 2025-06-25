package edu.edina.Libraries.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;

import edu.edina.Libraries.LinearMotion.IFeedForward;
import edu.edina.Libraries.LinearMotion.VoltageCompensation;
import edu.edina.Libraries.MotionControl.IMotionControlLinearMechanism;

public class ConstantPowerAction implements Action {
    private final double power;
    private final IMotionControlLinearMechanism mechanism;
    private final VoltageCompensation vc;
    private final IFeedForward feedFwd;

    public ConstantPowerAction(double power, IMotionControlLinearMechanism mechanism, VoltageCompensation optVolCom,
                               IFeedForward optFeedFwd) {
        this.power = power;
        this.mechanism = mechanism;
        vc = optVolCom;
        feedFwd = optFeedFwd;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        DualNum<Time> xv = mechanism.getPositionAndVelocity(false);
        double ff = feedFwd != null ? feedFwd.getPower(xv) : 0;
        double vcMult = vc != null ? vc.adjustPower(1) : 1;

        double p = (power + ff) * vcMult;
        mechanism.setPower(p);

        return true;
    }
}
