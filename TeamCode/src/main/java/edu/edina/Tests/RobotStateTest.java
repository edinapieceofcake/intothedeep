package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.edina.Libraries.Robot.RobotState;

@Autonomous
public class RobotStateTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotState rs = new RobotState(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            rs.update(telemetry);
            telemetry.update();
        }
    }
}
