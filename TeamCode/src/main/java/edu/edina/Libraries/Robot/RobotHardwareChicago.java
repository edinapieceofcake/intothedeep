package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

public class RobotHardwareChicago {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private Drivetrain drivetrain;
    private Arm2 arm;
    private Extension extension;
    private Lift2 lift;
    private RobotState robotState;
    private TelemetryPacket packet;

    public RobotHardwareChicago(HardwareMap hw) {
        packet = new TelemetryPacket();

        robotState = new RobotState(hw);

        drivetrain = new Drivetrain(hw);
        extension = new Extension(robotState, hw);
        arm = new Arm2(robotState, hw);
        lift = new Lift2(robotState, hw);
    }

    public void update() {
        robotState.update();

        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);

        drivetrain.update2();
    }

    public void highBasket() {
        runningActions.add(new ParallelAction(
                lift.moveLift(12),
                extension.moveExtension(10),
                arm.moveArm(60)
        ));
    }

    public void extend(double y) {
        if (Math.abs(y) < 0.1) {
            runningActions.add(
                    extension.holdPos()
            );
        } else {
            extension.cancelAction();
            extension.setPower(y);
        }
    }
}