package edu.edina.Libraries.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Moves the claw to the high basket.
public class MoveToHighBasket implements Action {

    // Robot hardware
    private RobotHardware robotHardware;

    // Initializes this.
    public MoveToHighBasket(RobotHardware robotHardware) {

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

    }

    // Runs this.
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {

        // Move the arm to the high basket position.
        robotHardware.setArmHighBasketAutoPosition();

        // Move the lift to the high basket position
        robotHardware.setLiftHighBasketPosition();

        // Use the high basket extension.
        robotHardware.setAutoHighBasketExtension();

        // Lower the wrist.
        robotHardware.lowerWrist();

        // Return indicating that the action is done.
        return false;

    }

}