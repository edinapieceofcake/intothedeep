package edu.edina.Libraries.Actions;

import static edu.edina.Libraries.Robot.LiftActions.k;
import static edu.edina.Libraries.Robot.LiftActions.g;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.RobotLog;

import edu.edina.Libraries.LinearMotion.LinearMotionController;
import edu.edina.Libraries.LinearMotion.SpringForce;
import edu.edina.Libraries.LinearMotion.VerticalExtensionMechanism;
import edu.edina.Libraries.Robot.RobotHardware;

@Controls(part = "lift")
public class RaiseLift implements Action {
    private final LinearMotionController verticalController;
    private final VerticalExtensionMechanism vertical;

    public RaiseLift(RobotHardware hw, double heightInches) {
        SpringForce springForce = new SpringForce(k, g);
        vertical = new VerticalExtensionMechanism(hw);

        verticalController = new LinearMotionController(vertical, springForce);
        verticalController.setTarget(heightInches);

        RobotLog.ii("RaiseLift", "init -- target %f in", heightInches);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        verticalController.run();
        return true;
    }
}