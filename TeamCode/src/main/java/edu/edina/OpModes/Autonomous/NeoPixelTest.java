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

        byte[] pixArray = new byte[90];

        for (int i = 1; i < pixArray.length; i += 3) {
            pixArray[i] = (byte) 0xff;
        }

        neoPixel.showColors(pixArray);

        while (opModeIsActive()) {

        }
    }
}