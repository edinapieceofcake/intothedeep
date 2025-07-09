package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.Robot.RobotHardwareChicago;

@Autonomous
public class BlobLocTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardwareChicago hw = new RobotHardwareChicago(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            hw.addSampleTelemetry(telemetry);
            telemetry.update();
        }
    }
}
