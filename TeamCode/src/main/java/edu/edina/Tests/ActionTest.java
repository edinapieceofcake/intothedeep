package edu.edina.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
public class ActionTest extends LinearOpMode {
    private List<Action> runningActions;
    private TestMechanism t;

    @Override
    public void runOpMode() throws InterruptedException {
        runningActions = new ArrayList<>();
        t = new TestMechanism(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();
        runToPos(300000);
        runningActions.add(new SequentialAction(
                new WaitForTime(3000),
                new InstantAction(() -> runToPos(0))
        ));

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            List<Action> oldActions = runningActions;
            runningActions = new ArrayList<>();
            for (Action action : oldActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    runningActions.add(action);
                }
            }

            dashboard.sendTelemetryPacket(packet);
        }
    }

    private void runToPos(double target) {
        runningActions.add(new RunToPositionAction(t, target));
    }
}