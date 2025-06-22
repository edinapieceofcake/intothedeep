package edu.edina.Libraries.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.RobotLog;

import edu.edina.Libraries.MotionControl.ICancelableAction;
import edu.edina.Libraries.MotionControl.IMotionControlLinearMechanism;

public class PidAction implements ICancelableAction {
    private final static String TAG = "PidAction";
    private final PIDController pid;
    private final IMotionControlLinearMechanism mechanism;
    private boolean done;

    public PidAction(double target, PidSettings pid, IMotionControlLinearMechanism mechanism) {
        this.pid = new PIDController(pid.p, pid.i, pid.d);
        this.pid.setSetPoint(target);
        this.mechanism = mechanism;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (done) {
            return false;
        }

        mechanism.setCurrentAction(this);

        double x = mechanism.getPosition(false);
        double power = pid.calculate(x);

        if (TAG != null) {
            RobotLog.ii(TAG, "%s: e_p = %.3f, e_v = %.3f --> power = %.3f",
                    mechanism.getName(),
                    pid.getPositionError(),
                    pid.getVelocityError(),
                    power);
        }

        mechanism.setPower(power);
        return true;
    }

    @Override
    public void cancel() {
        done = true;
    }
}

