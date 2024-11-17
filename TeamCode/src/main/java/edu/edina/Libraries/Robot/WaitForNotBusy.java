package edu.edina.Libraries.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Waits for the robot hardware to finish moving.
public class WaitForNotBusy implements Action {

    // Robot hardware
    private RobotHardware robotHardware;

    // Update value
    private boolean update;

    // Initializes this.
    public WaitForNotBusy(RobotHardware robotHardware, boolean update) {

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

    }

    // Runs this.
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {

        // If we are updating...
        if(update) {

            // Update the robot hardware.
            robotHardware.update();

        }

        // Determine whether the robot hardware is busy.
        boolean isBusy = robotHardware.isArmBusy() || robotHardware.isLiftBusy() || robotHardware.isSlideBusy();

        // Return the result.
        return isBusy;

    }

}