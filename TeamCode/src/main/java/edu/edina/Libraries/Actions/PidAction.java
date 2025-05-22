package edu.edina.Libraries.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;

import edu.edina.Libraries.MotionControl.IMotionControlLinearMechanism;

public class PidAction implements Action {
    private final PIDController pid;
    private final IMotionControlLinearMechanism mechanism;

    public PidAction(double target, PidSettings pid, IMotionControlLinearMechanism mechanism) {
        this.pid = new PIDController(pid.p, pid.i, pid.d);
        this.pid.setSetPoint(target);
        this.mechanism = mechanism;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        double x = mechanism.getPosition(false);
        double power = pid.calculate(x);
        mechanism.setPower(power);
        return true;
    }
}
