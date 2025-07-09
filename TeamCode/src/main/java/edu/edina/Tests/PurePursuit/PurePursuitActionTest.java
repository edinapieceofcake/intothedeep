package edu.edina.Tests.PurePursuit;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.edina.Libraries.Robot.RobotHardwareChicago;

@Autonomous
@Config
public class PurePursuitActionTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardwareChicago hw = new RobotHardwareChicago(hardwareMap);
        waitForStart();
        hw.addPath(new Vector2d[]{
                new Vector2d(0, 0),
                new Vector2d(10, 0),
        }, 0);
        while (opModeIsActive()) {
            hw.update(telemetry);
            telemetry.update();
        }
    }
}