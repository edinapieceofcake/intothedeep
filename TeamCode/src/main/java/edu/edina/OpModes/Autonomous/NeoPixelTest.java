package edu.edina.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.edina.Libraries.Robot.NeoPixelDriverDevice;

@Autonomous
public class NeoPixelTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        NeoPixelDriverDevice neoPixel = hardwareMap.get(NeoPixelDriverDevice.class, "neopixel_driver");

        waitForStart();

        neoPixel.showColors();

        while (opModeIsActive()) {

        }
    }
}