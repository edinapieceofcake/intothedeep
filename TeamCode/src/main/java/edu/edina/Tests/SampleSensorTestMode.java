package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.SampleColor;
import edu.edina.Libraries.Robot.SampleSensor;

@TeleOp
@Disabled
public class SampleSensorTestMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleSensor s = new SampleSensor(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (s.detectSampleColor() == SampleColor.BLUE)
                telemetry.addLine("Blue");
            else if (s.detectSampleColor() == SampleColor.RED)
                telemetry.addLine("Red");
            else if (s.detectSampleColor() == SampleColor.RED)
                telemetry.addLine("Red");
            else
                telemetry.addLine("No Sample");

            telemetry.update();
        }
    }
}
