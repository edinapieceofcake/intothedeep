package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.ArrayList;
import java.util.List;

public class ActionList {
    private List<Action> runningActions;

    public ActionList() {
        runningActions = new ArrayList<>();
    }

    public void clear() {
        runningActions.clear();
    }

    public void add(Action a) {
        runningActions.add(a);
    }

    public void remove(Action a) {
        runningActions.remove(a);
    }

    public void run(TelemetryPacket packet) {
        List<Action> runList = runningActions;
        runningActions = new ArrayList<>();

        for (Action action : runList) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                runningActions.add(action);
            }
        }
    }
}
