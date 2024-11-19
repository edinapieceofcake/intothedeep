package edu.edina.Libraries.Robot;

import static edu.edina.Libraries.Robot.Lift.DESCEND_POSITION;
import static edu.edina.Libraries.Robot.Lift.MINIMUM_POSITION;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Descends the robot.
@Config
public class Descend implements Action {

    // Increment
    public static int INCREMENT = 50;

    // Current position
    private int currentPosition;

    // Robot hardware
    private RobotHardware robotHardware;

    // Initializes this.
    public Descend(RobotHardware robotHardware) {

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

        // Get the lift's current position.
        currentPosition = robotHardware.getCurrentLiftPosition();

    }

    // Runs this.
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {

        // Move the arm to the ascent position.
        robotHardware.setArmAscentPosition();

        // If the lift is in the descend position...
        if(currentPosition >= DESCEND_POSITION) {

            // Return indicating that this is done.
            return false;

        }

        // Increment the current position.
        currentPosition = Math.min(currentPosition + INCREMENT, DESCEND_POSITION);

        // Set the lift's position.
        robotHardware.setLiftPosition(currentPosition);

        // Return indicating that this is still running.
        return true;

    }

}