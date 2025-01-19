package edu.edina.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;

import java.util.ArrayList;
import java.util.List;

import edu.edina.Libraries.Actions.RunToPositionAction;
import edu.edina.Libraries.LinearMotion.TestMechanism;
import edu.edina.Libraries.Robot.WaitForTime;

@TeleOp
public class ActionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        List<Action> runningActions = new ArrayList<>();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TestMechanism t = new TestMechanism(hardwareMap);

        waitForStart();
        runningActions.add(new RunToPositionAction(t, 300000));
        runningActions.add(new SequentialAction(
                new WaitForTime(3000),
                new RunToPositionAction(t, 0)
        ));

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }

            runningActions = newActions;
            dashboard.sendTelemetryPacket(packet);
        }
    }
}