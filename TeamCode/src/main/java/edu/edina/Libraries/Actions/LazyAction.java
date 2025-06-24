package edu.edina.Libraries.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.function.Supplier;

public class LazyAction implements Action {
    private final Supplier<Action> actionConstructor;
    private Action action;

    public LazyAction(Supplier<Action> actionConstructor) {
        this.actionConstructor = actionConstructor;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (action == null)
            action = actionConstructor.get();

        return action.run(telemetryPacket);
    }
}
