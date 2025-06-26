package edu.edina.Libraries.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.RobotLog;

public class LogAction implements Action {
    private final String tag, msg;

    public LogAction(String tag, String msg) {
        this.tag = tag;
        this.msg = msg;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        RobotLog.ii(tag, msg);
        return false;
    }
}
