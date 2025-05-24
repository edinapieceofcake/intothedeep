package edu.edina.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;

import edu.edina.Libraries.Robot.Lift2;

@TeleOp(name = "TeleOp Main \uD83C\uDF82", group = "a")
public class TeleOpChicago extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryPacket packet = new TelemetryPacket();

        Lift2.Mechanism m = new Lift2.Mechanism(hardwareMap);

        Lift2 lift = new Lift2(m);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.y) {
                runningActions.clear();
                runningActions.add(lift.moveLift(10));
            }

            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);
        }
    }
}
