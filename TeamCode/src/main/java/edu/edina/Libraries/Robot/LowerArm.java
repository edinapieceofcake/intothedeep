package edu.edina.Libraries.Robot;

import static edu.edina.Libraries.Robot.Arm.GROUND_POSITION;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Lowers the arm.
@Config
public class LowerArm implements Action {

    // Position increment
    public static int POSITION_INCREMENT = 100;

    // Current position
    private int currentPosition;

    // Initialized value
    private boolean initialized;

    // Robot hardware
    private RobotHardware robotHardware;

    // Initializes this.
    public LowerArm(RobotHardware robotHardware) {

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

    }

    // Runs this.
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {

        // If this is uninitialized...
        if (!initialized) {

            // Get the arm's current position.
            currentPosition = robotHardware.getCurrentArmPosition();

            // Remember that this is initialized.
            initialized = true;

            // Return indicating that this is still running.
            return true;

        }

        // If the arm has not reached the ground...
        if(currentPosition > GROUND_POSITION) {

            // Decrement the current position.
            currentPosition = Math.max(currentPosition - POSITION_INCREMENT, GROUND_POSITION);

            // Set the arm's position.
            robotHardware.setArmPosition(currentPosition);

            // Return indicating that this is still running.
            return true;

        }

        // Return indicating that this is done.
        return false;

    }

}