package edu.edina.Libraries.Robot;

import static edu.edina.Libraries.Robot.Arm.GROUND_POSITION;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Lowers the arm.
@Config
public class LowerArm implements Action {

    // Fast increment
    public static int FAST_INCREMENT = 200;

    // Slow increment
    public static int SLOW_INCREMENT = 100;

    // Current position
    private int currentPosition;

    // Increment
    private int increment;

    // Robot hardware
    private RobotHardware robotHardware;

    // Initializes this.
    public LowerArm(RobotHardware robotHardware) {

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

        // Get the arm's current position.
        currentPosition = robotHardware.getCurrentArmPosition();

        // Get an appropriate increment.
        if(robotHardware.isLiftInGroundPosition() && robotHardware.isSlideFullyRetracted()) {
            increment = FAST_INCREMENT;
        }
        else {
            increment = SLOW_INCREMENT;
        }

    }

    // Runs this.
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {

        // If the arm is in the ground position...
        if(currentPosition <= GROUND_POSITION) {

            // Return indicating that this is done.
            return false;

        }

        // Decrement the current position.
        currentPosition = Math.max(currentPosition - increment, GROUND_POSITION);

        // Set the arm's position.
        robotHardware.setArmPosition(currentPosition);

        // Return indicating that this is still running.
        return true;

    }

}