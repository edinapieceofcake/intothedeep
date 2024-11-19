package edu.edina.Libraries.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Moves the arm.
@Config
public class MoveArm implements Action {

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

    // Target position
    private int targetPosition;

    // Initializes this.
    public MoveArm(RobotHardware robotHardware, int targetPosition) {

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

        // Get the target position.
        this.targetPosition = targetPosition;

        // Get the arm's current position.
        currentPosition = robotHardware.getCurrentArmPosition();

        // Get an appropriate increment.
        if(robotHardware.isLiftInGroundPosition() && robotHardware.isSlideFullyRetracted()) {
            increment = FAST_INCREMENT;
        }
        else {
            increment = SLOW_INCREMENT;
        }

        // If we are lowering the arm...
        if(currentPosition > targetPosition) {

            // Negate the increment.
            increment = -increment;

        }

    }

    // Runs this.
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {

        // Determine whether the arm is in position.
        boolean inPosition =
                (increment > 0 && currentPosition >= targetPosition) ||
                        (increment < 0 && currentPosition <= targetPosition);

        // If the arm is in position...
        if(inPosition) {

            // Return indicating that this is done.
            return false;

        }

        // Update the current position.
        if(increment > 0) {
            currentPosition = Math.min(currentPosition + increment, targetPosition);
        }
        else {
            currentPosition = Math.max(currentPosition + increment, targetPosition);
        }

        // Set the arm's position.
        robotHardware.setArmPosition(currentPosition);

        // Return indicating that this is still running.
        return true;

    }

}