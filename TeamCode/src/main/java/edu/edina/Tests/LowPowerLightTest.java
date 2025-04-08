package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.Robot.Light;
import edu.edina.Libraries.Robot.SampleColor;
import edu.edina.Libraries.Robot.SampleSensor;

@TeleOp
@Disabled
public class LowPowerLightTest extends LinearOpMode {
    private Light light;
    private SampleSensor sampleSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        sampleSensor = new SampleSensor(hardwareMap);
        light = new Light(hardwareMap, sampleSensor);

        waitForStart();

        while (opModeIsActive()) {
            light.update(false, false);
        }
    }
}