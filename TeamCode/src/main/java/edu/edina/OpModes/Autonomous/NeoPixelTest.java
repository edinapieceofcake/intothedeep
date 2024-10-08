package edu.edina.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Random;

import edu.edina.Libraries.Robot.NeoPixelDriverDevice;

@Autonomous
public class NeoPixelTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        NeoPixelDriverDevice neoPixel = hardwareMap.get(NeoPixelDriverDevice.class, "neopixel_driver");

        waitForStart();

        byte[] pixArray = new byte[60];

        Random r = new Random();

        while (opModeIsActive()) {
            for (int i = 0; i < pixArray.length; i += 1) {
                pixArray[i] = (byte) r.nextInt();
            }

            neoPixel.showColors(pixArray);

            sleep(100);
        }
    }
}