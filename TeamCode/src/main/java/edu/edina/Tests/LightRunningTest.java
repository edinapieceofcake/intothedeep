package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.Robot.Light;
import edu.edina.Libraries.Robot.SampleSensor;

@TeleOp
public class LightRunningTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleSensor sampleSensor = new SampleSensor(hardwareMap);
        Light light = new Light(hardwareMap, sampleSensor);

        waitForStart();

        while (opModeIsActive()) {
            light.update(false, true);

            sampleSensor.addTelemetry(telemetry);
            telemetry.update();
        }
    }
}