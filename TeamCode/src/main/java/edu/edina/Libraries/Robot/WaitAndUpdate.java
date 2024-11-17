package edu.edina.Libraries.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

// Waits for a specified duration.
public class WaitAndUpdate implements Action {

    // Initialized value
    private boolean initialized;

    // Duration in milliseconds
    private double milliseconds;

    // Robot hardware
    private RobotHardware robotHardware;

    // Timer
    private ElapsedTime timer;

    // Update
    private boolean update;

    // Initialzies this.
    public WaitAndUpdate(RobotHardware robotHardware, double milliseconds, boolean update)
    {

        // Remember the duration in milliseconds.
        this.milliseconds = milliseconds;

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

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

        // Determine whether we are waiting.
        boolean waiting = timer.milliseconds() < milliseconds;

        // Return the result.
        return waiting;

    }

}