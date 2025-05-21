package edu.edina.Tests.PurePursuit;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.edina.Libraries.PurePursuit.PurePursuitAction;
import edu.edina.Libraries.Robot.RobotHardware;

@Autonomous
@Config
public class PurePursuitActionTest extends LinearOpMode {
    public static double TGT = 2;
    public static double MAX = 15;
//    public static double ANG_MAX = 15;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);

        Vector2d[] v = new Vector2d[]{
                new Vector2d(0, 0),
                new Vector2d(20, 0),
                new Vector2d(20, 5)
        };

        PurePursuitAction action = new PurePursuitAction(v, TGT, MAX, 5, hw);

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket pkt = new TelemetryPacket();
            boolean keepRunning = action.run(pkt);
            if (!keepRunning)
                break;
        }

        while (opModeIsActive()) {
            telemetry.addLine("done");
            telemetry.update();
        }
    }
}