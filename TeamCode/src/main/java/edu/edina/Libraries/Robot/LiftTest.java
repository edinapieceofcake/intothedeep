package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LiftTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);
        LiftActions liftActions = new LiftActions(hw);
        TelemetryPacket telemetryPacket = new TelemetryPacket();
        FtcDashboard ftcDashboard = FtcDashboard.getInstance();

        waitForStart();

        Action raise = new SequentialAction(
                liftActions.setTarget(14),
                liftActions.holdPos(14)
        );

        Action lower = new SequentialAction(
                liftActions.setTarget(0.3),
                liftActions.holdPos(0)
        );

        while (opModeIsActive()) {
            telemetryPacket.put("current draw", liftActions.getVMech().getCurrent());
            telemetryPacket.put("position", liftActions.getVMech().getPositionAndVelocity(false).get(0));

            if (gamepad1.dpad_up) {
                raise.run(new TelemetryPacket());
            }
            if (gamepad1.dpad_down) {
                lower.run(new TelemetryPacket());
            }

            ftcDashboard.sendTelemetryPacket(telemetryPacket);
        }
    }
}