package edu.edina.Tests.PurePursuit;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.edina.Libraries.PurePursuit.Path;
import edu.edina.Libraries.Robot.RobotHardwareChicago;

@Autonomous
@Config
public class PurePursuitActionTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardwareChicago hw = new RobotHardwareChicago(
                hardwareMap,
                new Pose2d(new Vector2d(0, 0), 0));

        while (opModeInInit()) {
            hw.initUpdate(telemetry);
            telemetry.update();
        }

        hw.addPath(new Path(
                new Vector2d[]{
                        new Vector2d(0, 0),
                        new Vector2d(40, 0),
                        new Vector2d(40, 10)
                }).withName("pp-test.csv").withHeading(90));

        while (opModeIsActive()) {
            hw.update(telemetry);
            telemetry.update();
        }
    }
}