package edu.edina.OpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.edina.Libraries.Robot.RobotHardwareChicago;

@Config
@Autonomous
public class AutoSampleAllianceSide extends LinearOpMode {
    public static double tgt = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardwareChicago hw = new RobotHardwareChicago(hardwareMap);

        Vector2d[] v = new Vector2d[]{
                new Vector2d(0, 0),
                new Vector2d(20, 0),
                new Vector2d(20, -20)
        };

        waitForStart();

        hw.addPath(v, tgt);

        while (opModeIsActive()) {
            hw.update(telemetry);
            telemetry.update();
        }
    }
}