package edu.edina.Libraries.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;

import edu.edina.Libraries.LinearMotion.IFeedForward;
import edu.edina.Libraries.LinearMotion.VoltageCompensation;
import edu.edina.Libraries.MotionControl.ILinearMechanism;

public class SinglePowerPid implements Action {
    private double power, target, tolerance;
    private IFeedForward feedForward;
    private VoltageCompensation voltageCompensation;
    private ILinearMechanism mechanism;

    public SinglePowerPid(double power, double target, double tolerance, IFeedForward feedForward, ILinearMechanism mechanism, VoltageCompensation voltageCompensation) {
        this.power = power;
        this.target = target;
        this.feedForward = feedForward;
        this.voltageCompensation = voltageCompensation;
        this.mechanism = mechanism;
        this.tolerance = tolerance;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        DualNum<Time> xv = mechanism.getPositionAndVelocity(false);
        double x = xv.value();
        double ff = feedForward != null ? feedForward.getPower(xv) : 0;
        double vcMult = voltageCompensation != null ? voltageCompensation.adjustPower(1) : 1;
        double basePower = 0;

        if (x + tolerance < target) {
            basePower += power;
        } else if (x - tolerance > target) {
            basePower -= power;
        }

        double power = (basePower + ff) * vcMult;
        mechanism.setPower(power);

        return true;
    }
}