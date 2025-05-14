package edu.edina.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.LinearMotion.IMotionControlLinearMechanism;
import edu.edina.Libraries.Robot.Arm2;
import edu.edina.Libraries.Robot.MotionControlAction;

@TeleOp
@Config
public class MotionControlTest extends LinearOpMode {
    public static double TARGET_POS = 45;
    public static double TARGET_VEL = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        IMotionControlLinearMechanism mechanism = new Arm2.Mechanism(hardwareMap);
        MotionControlAction action = new MotionControlAction(TARGET_POS, TARGET_VEL, mechanism);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData("target pos", TARGET_POS);
        telemetry.addData("target vel", TARGET_VEL);
        telemetry.addLine("waiting for start to move");
        telemetry.update();

        while (opModeInInit()) {
            TelemetryPacket p = new TelemetryPacket();
            action.addTelemetry(p);
            dashboard.sendTelemetryPacket(p);
        }

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket p = new TelemetryPacket();
            boolean keepRunning = action.run(p);
            dashboard.sendTelemetryPacket(p);

            if (!keepRunning)
                break;
        }

        while (opModeIsActive()) {
            TelemetryPacket p = new TelemetryPacket();
            action.addTelemetry(p);
            dashboard.sendTelemetryPacket(p);

            telemetry.addLine("done");
            telemetry.update();
        }
    }
}
