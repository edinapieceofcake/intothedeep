package edu.edina.Libraries.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;

import edu.edina.Libraries.MotionControl.ICancelableAction;
import edu.edina.Libraries.MotionControl.IMotionControlLinearMechanism;

public class PidAction implements ICancelableAction {
    private final PIDController pid;
    private final IMotionControlLinearMechanism mechanism;
    private boolean done;

    public PidAction(double target, PidSettings pid, IMotionControlLinearMechanism mechanism) {
        this.pid = new PIDController(pid.p, pid.i, pid.d);
        this.pid.setSetPoint(target);
        this.mechanism = mechanism;

        mechanism.setCurrentAction(this);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        double x = mechanism.getPosition(false);
        double power = pid.calculate(x);
        mechanism.setPower(power);
        return !done;
    }

    @Override
    public void cancel() {
        done = true;
    }
}
