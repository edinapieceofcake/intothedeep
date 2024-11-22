package edu.edina.Libraries.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Moves the claw to the ground.
public class MoveToGround implements Action {

    // Robot hardware
    private RobotHardware robotHardware;

    // Initializes this.
    public MoveToGround(RobotHardware robotHardware) {

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

    }

    // Runs this.
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {

        // Move the arm to the ground position.
        robotHardware.setArmGroundPosition();

        // Move the lift to the ground position
        robotHardware.setLiftGroundPosition();

        // Fully retract the slide.
        robotHardware.setMinimumExtension();

        // Return indicating that the action is done.
        return false;

    }

}