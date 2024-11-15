package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.Robot.GamePadClick;
import edu.edina.Libraries.Robot.RobotHardware;

@TeleOp
public class BoundingBoxTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        GamePadClick c = new GamePadClick(gamepad1);
        RobotHardware hw = new RobotHardware(this);

        while (opModeIsActive()) {
            c.read();

            if (c.a)
                hw.toggleWrist();

            hw.getFailsafe().apply();

            hw.getFailsafe().addToTelemetry(telemetry);
            telemetry.update();
        }
    }
}
