package edu.edina.Libraries.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.RobotLog;

public class WaitWhile implements Action {
    private final String name;
    private final Condition condition;

    public WaitWhile(String name, Condition condition) {
        this.name = name;
        this.condition = condition;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        boolean value = condition.run();
        RobotLog.ii("WaitAction", "while %s = %s", name, value);
        return value;
    }
}
