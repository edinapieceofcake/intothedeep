package edu.edina.Libraries.Robot;

import static edu.edina.Libraries.Robot.Lift.ASCEND_POSITION;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Ascends the robot.
@Config
public class Ascend implements Action {

    // Increment
    public static int INCREMENT = 100;

    // Current position
    private int currentPosition;

    // Robot hardware
    private RobotHardware robotHardware;

    // Initializes this.
    public Ascend(RobotHardware robotHardware) {

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

        // If the lift is in the ascend position...
        if(currentPosition <= ASCEND_POSITION) {

            // Return indicating that this is done.
            return false;

        }

        // Decrement the current position.
        currentPosition = Math.max(currentPosition - INCREMENT, ASCEND_POSITION);

        // Set the lift's position.
        robotHardware.setLiftPosition(currentPosition);

        // Return indicating that this is still running.
        return true;

    }

}