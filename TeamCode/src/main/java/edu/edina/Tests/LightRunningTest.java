package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import edu.edina.Libraries.Robot.Light;
import edu.edina.Libraries.Robot.SampleSensor;

@TeleOp
public class LightRunningTest extends LinearOpMode {
    private Gamepad currentGamepad1 = new Gamepad();

    // Previous gamepad
    private Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void runOpMode() throws InterruptedException {
        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        SampleSensor sampleSensor = new SampleSensor(hardwareMap);
        Light light = new Light(hardwareMap, sampleSensor);

        boolean wave = false;
        boolean waveEffect = false;
        boolean sampleColor = true;

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.a && !previousGamepad1.a) {
                wave = !wave;
                if (wave) {
                    sampleColor = false;
                    waveEffect = true;
                } else {
                    sampleColor = true;
                    waveEffect = false;
                }
            }

            light.update(waveEffect, sampleColor);

            telemetry.update();
        }
    }
}