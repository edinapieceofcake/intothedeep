package edu.edina.Tests;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

import edu.edina.Libraries.Robot.RobotHardware;

@TeleOp
public class SpecimenParkingTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);

        List<Action> runningActions = new ArrayList<>();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a && runningActions.size() == 1) {
                // add parking
            }

            TelemetryPacket packet = new TelemetryPacket();
            List<Action> newActions = new ArrayList<>();

            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());

                if (action.run(packet)) {
                    newActions.add(action);
                }
            }

            runningActions = newActions;
            Pose2d pose = hw.getOdometry().getPoseEstimate();

            telemetry.addData("position: ", "x = %.2f y = %.2f", pose.position.x, pose.position.y);
            telemetry.addData("heading", "%.2f (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();
        }
    }
}
