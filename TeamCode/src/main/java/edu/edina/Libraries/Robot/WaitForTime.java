package edu.edina.Libraries.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

// Waits for a specified time.
public class WaitForTime implements Action {

    // Initialized value
    private boolean initialized;

    // Duration in milliseconds
    private double milliseconds;

    // Timer
    private ElapsedTime timer;

    // Initializes this.
    public WaitForTime(double milliseconds)
    {

        // Remember the duration in milliseconds.
        this.milliseconds = milliseconds;

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

        // Determine whether we are waiting.
        boolean waiting = timer.milliseconds() < milliseconds;

        // Return the result.
        return waiting;

    }

}