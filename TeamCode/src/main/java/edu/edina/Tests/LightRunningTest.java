package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.Robot.Light;
import edu.edina.Libraries.Robot.SampleSensor;

@TeleOp
public class LightRunningTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Light light = new Light(hardwareMap, new SampleSensor(hardwareMap));

        waitForStart();

        while (opModeIsActive()) {
            light.update(false, true);
        }
    }
}