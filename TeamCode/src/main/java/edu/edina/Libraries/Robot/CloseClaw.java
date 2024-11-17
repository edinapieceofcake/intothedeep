package edu.edina.Libraries.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Closes the claw.
public class CloseClaw implements Action {

    // Robot hardware
    private RobotHardware robotHardware;

    // Initializes this.
    public CloseClaw(RobotHardware robotHardware) {

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

    }

    // Runs this.
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {

        // Close the claw.
        robotHardware.closeClaw();

        // Return indicating that the action is done.
        return false;

    }

}