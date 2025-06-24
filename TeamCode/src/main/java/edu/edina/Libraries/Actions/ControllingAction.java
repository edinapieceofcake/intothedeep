package edu.edina.Libraries.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class ControllingAction implements Action {
    private final Action innerAction;
    private final ControllingActionManager manager;
    private boolean keepRunning;

    public ControllingAction(Action innerAction, ControllingActionManager manager) {
        this.innerAction = innerAction;
        this.manager = manager;
        keepRunning = true;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (keepRunning) {
            keepRunning = innerAction.run(telemetryPacket);
            manager.updateControllingAction(this, keepRunning);
        }
        return keepRunning;
    }

    public void cancel() {
        keepRunning = false;
    }
}
