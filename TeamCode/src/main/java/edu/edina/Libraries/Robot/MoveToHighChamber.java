package edu.edina.Libraries.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Moves the claw to the high chamber.
public class MoveToHighChamber implements Action {

    // Robot hardware
    private RobotHardware robotHardware;

    // Initializes this.
    public MoveToHighChamber(RobotHardware robotHardware) {

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

    }

    // Runs this.
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {

        // Lower the wrist.
        robotHardware.lowerWrist();

        // Move the arm to the high chamber position.
        robotHardware.setArmHighChamberPosition();

        // Move the lift to the ground position
        robotHardware.setLiftGroundPosition();

        // Use the high chamber extension.
        robotHardware.setMinimumExtension();

        // Return indicating that the action is done.
        return false;

    }

}