package edu.edina.Tests;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

import edu.edina.Libraries.Actions.ActionListOld;
import edu.edina.Libraries.Actions.Condition;
import edu.edina.Libraries.Actions.Conditions;
import edu.edina.Libraries.Actions.OdometryUpdater;
import edu.edina.Libraries.Actions.RaiseLift;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Actions.SpecimenPark;

@TeleOp
@Disabled
public class SpecimenParkingTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);

        List<Action> runningActions = new ArrayList<>();
        runningActions.add(new OdometryUpdater(hw));

        waitForStart();

        Condition autoCondition = null;

        while (opModeIsActive()) {
            // check if driver wants to start auto drive
            Condition sp = new Conditions.SpecimenPark(this);

            if (sp.run()) {
                if (ActionListOld.canAddAction(SpecimenPark.class, runningActions)) {
                    runningActions.add(new SpecimenPark(hw, sp));
                }

                if (ActionListOld.canAddAction(RaiseLift.class, runningActions)) {
                    runningActions.add(new RaiseLift(hw, 14));
                }
            }

            // check if still in manual driving mode
            if (ActionListOld.canControlPart("drivetrain", runningActions)) {
                hw.drivetrain.update();
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