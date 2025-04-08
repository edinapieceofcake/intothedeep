package edu.edina.Libraries.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class ContinuousAction implements Action {
    private ContinuousFunc f;

    public ContinuousAction(ContinuousFunc f) {
        this.f = f;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return f.run();
    }
}