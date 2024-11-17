package edu.edina.Libraries.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Opens the claw.
public class OpenClaw implements Action {

    // Robot hardware
    private RobotHardware robotHardware;

    // Initializes this.
    public OpenClaw(RobotHardware robotHardware) {

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

    }

    // Runs this.
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {

        // Open the claw.
        robotHardware.openClaw();

        // Return indicating that the action is done.
        return false;

    }

}