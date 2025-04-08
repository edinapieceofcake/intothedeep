package edu.edina.Libraries.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import edu.edina.Libraries.Robot.Odometry;
import edu.edina.Libraries.Robot.RobotHardware;

public class OdometryUpdater implements Action {
    private final Odometry odometry;

    public OdometryUpdater(RobotHardware hw) {
        odometry = hw.getOdometry();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        odometry.update();
        return true;
    }
}
