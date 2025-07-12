package edu.edina.Libraries.PurePursuit;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;

import edu.edina.Libraries.Robot.Drivetrain;
import edu.edina.Libraries.Robot.RobotState;

public class BrakeAction implements Action {
    private final RobotState robotState;
    private final Drivetrain drivetrain;
    private final double minSpeed;

    public BrakeAction(RobotState robotState, Drivetrain drivetrain, double minSpeed) {
        this.robotState = robotState;
        this.drivetrain = drivetrain;
        this.minSpeed = minSpeed;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Pose2dDual<Time> poseDual = robotState.getCurrentPoseDual();
        PoseVelocity2d v = poseDual.velocity().value();

        if (v.linearVel.norm() > minSpeed) {
            drivetrain.brake();
            return true;
        } else {
            drivetrain.coast();
            return false;
        }
    }
}
