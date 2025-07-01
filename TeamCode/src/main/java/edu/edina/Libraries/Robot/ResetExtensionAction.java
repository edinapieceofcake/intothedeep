package edu.edina.Libraries.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

public class ResetExtensionAction implements Action {
    public static double TIME_RUNNING = 100;

    private ElapsedTime timer;
    private RobotState rS;
    private Extension.Mechanism mechanism;

    private double pos;

    public ResetExtensionAction(RobotState rS, Extension.Mechanism mechanism) {
        this.rS = rS;
        timer = new ElapsedTime();
        this.mechanism = mechanism;
        pos = rS.getExtensionPos();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        mechanism.setPower(-1);
        pos = Math.min(pos, rS.getExtensionPos());

        if (timer.milliseconds() > TIME_RUNNING) {
            mechanism.setPower(0);
            rS.resetExtension(pos);
            return false;
        } else {
            return true;
        }
    }
}