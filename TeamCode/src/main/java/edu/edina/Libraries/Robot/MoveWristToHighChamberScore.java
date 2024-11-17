package edu.edina.Libraries.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class MoveWristToHighChamberScore implements Action {

    // Robot hardware
    private RobotHardware robotHardware;

    // Initializes this.
    public MoveWristToHighChamberScore(RobotHardware robotHardware) {

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

    }

    // Runs this.
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {

        // Move the wrist to the high chamber score position.
        robotHardware.moveWristToHighChamberScore();

        // Return indicating that the action is done.
        return false;

    }

}