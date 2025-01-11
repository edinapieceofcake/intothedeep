package edu.edina.Libraries.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import edu.edina.Libraries.Robot.Drivetrain;
import edu.edina.Libraries.Robot.RobotHardware;

public class KillSwitchAction implements Action {
    private final Condition shouldKill;
    private final RobotHardware robotHardware;


    public KillSwitchAction(RobotHardware robotHardware, Condition shouldKill) {
        this.shouldKill = shouldKill;
        this.robotHardware = robotHardware;


    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (shouldKill.run()) {
            robotHardware.clearActions();
            robotHardware.drivetrain.stop();
            // resuturn false becuase we are done checking
            return false;
        } else {
            // reture true becuase thsi value we want the killswitchaction.run to be called again
            return true;
        }

    }
}
