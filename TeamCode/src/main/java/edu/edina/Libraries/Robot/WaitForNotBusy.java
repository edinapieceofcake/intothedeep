package edu.edina.Libraries.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

// Waits for the robot hardware to finish moving.
public class WaitForNotBusy implements Action {

    // Initialized value
    private boolean initialized;

    // Duration in milliseconds
    private double milliseconds;

    // Robot hardware
    private RobotHardware robotHardware;

    // Timer
    private ElapsedTime timer;

    // Update value
    private boolean update;

    // Initializes this.
    public WaitForNotBusy(RobotHardware robotHardware, double milliseconds, boolean update) {

        // Remember the duration in milliseconds.
        this.milliseconds = milliseconds;

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

        // Remember the update value.
        this.update = update;

    }

    // Runs this.
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {

        // If this is not initialized...
        if (!initialized) {

            // Start a timer.
            timer = new ElapsedTime();

            // Remember that this is initialized.
            initialized = true;

        }

        // If we are updating...
        if(update) {

            // Update the robot hardware.
            robotHardware.update();

        }

        // If we timed out...
        if(timer.milliseconds() > milliseconds) {

            // Return indicating that the action is done.
            return false;

        }

        // Determine whether the robot hardware is busy.
        boolean isBusy = robotHardware.isArmBusy() || robotHardware.isLiftBusy() || robotHardware.isSlideBusy();

        // Return the result.
        return isBusy;

    }

}