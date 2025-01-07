package edu.edina.Libraries.Actions;

import static edu.edina.Libraries.Robot.LiftTest.g;
import static edu.edina.Libraries.Robot.LiftTest.k;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import edu.edina.Libraries.LinearMotion.LinearMotionController;
import edu.edina.Libraries.LinearMotion.SpringForce;
import edu.edina.Libraries.LinearMotion.VerticalExtensionMechanism;
import edu.edina.Libraries.Robot.RobotHardware;

public class RaiseLift implements Action {
    private final LinearMotionController verticalController;

    public RaiseLift(RobotHardware hw, double heightInches) {
        SpringForce springForce = new SpringForce(k, g);
        VerticalExtensionMechanism vertical = new VerticalExtensionMechanism(hw);

        verticalController = new LinearMotionController(vertical, springForce);
        verticalController.setTarget(heightInches);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        verticalController.run();

        return true;
    }
}