package edu.edina.Libraries.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.RobotLog;

import edu.edina.Libraries.LinearMotion.IFeedForward;
import edu.edina.Libraries.LinearMotion.VoltageCompensation;
import edu.edina.Libraries.MotionControl.ICancelableAction;
import edu.edina.Libraries.MotionControl.IMotionControlLinearMechanism;

public class PidAction implements ICancelableAction {
    private final static String TAG = "PidAction";
    private final PIDController pid;
    private final IMotionControlLinearMechanism mechanism;
    private boolean done;
    private final VoltageCompensation vc;
    private final IFeedForward feedFwd;

    public PidAction(double target, PidSettings pid, IMotionControlLinearMechanism mechanism, VoltageCompensation optVolCom,
                     IFeedForward optFeedFwd) {
        this.pid = new PIDController(pid.p, pid.i, pid.d);
        this.pid.setSetPoint(target);
        this.mechanism = mechanism;
        vc = optVolCom;
        feedFwd = optFeedFwd;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (done) {
            return false;
        }

        DualNum<Time> xv = mechanism.getPositionAndVelocity(false);
        double x = mechanism.getPosition(false);
        double pidPower = pid.calculate(x);
        double ff = feedFwd != null ? feedFwd.getPower(xv) : 0;
        double vcMult = vc != null ? vc.adjustPower(1) : 1;

        double p = (pidPower + ff) * vcMult;

        if (TAG != null) {
            RobotLog.ii(TAG, "%s: e_p = %.3f, e_v = %.3f --> power = %.3f = (%.3f + %.3f)*%.3f",
                    mechanism.getName(),
                    pid.getPositionError(),
                    pid.getVelocityError(),
                    p, pidPower, ff, vcMult);
        }

        mechanism.setPower(p);

        return true;
    }

    @Override
    public void cancel() {
        done = true;
    }
}

