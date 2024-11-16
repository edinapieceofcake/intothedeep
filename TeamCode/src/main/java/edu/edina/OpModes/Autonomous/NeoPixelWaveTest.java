package edu.edina.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import edu.edina.Libraries.Robot.NeoPixelDriverDevice;

@Autonomous
@Disabled
public class NeoPixelWaveTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        NeoPixelDriverDevice neoPixel = hardwareMap.get(NeoPixelDriverDevice.class, "neopixel_driver");

        waitForStart();

        byte[] pixArray = new byte[NeoPixelDriverDevice.NUM_BYTES];
        int numPixel = NeoPixelDriverDevice.NUM_PIXELS;

        ElapsedTime t = new ElapsedTime();

        while (opModeIsActive()) {
            double x;

            double blueTime = Math.sin(t.seconds() * 1.5);
            double a = 1/40.0;
            double greenTime = t.seconds() * 0.5;
            double greenFrequency = 1;

            for (int n = 0; n < numPixel; n++) {
                int blueByte = n * 3 + 2;
                x = n * a;
                pixArray[blueByte] = toByte(Math.sin(blueTime + x), -1, 1, 10, 200);
            }

            for (int n = 0; n < numPixel; n++) {
                int greenByte = n * 3;
                x = greenFrequency * n;
                pixArray[greenByte] = toByte(Math.sin(blueTime + x), -1, 1, 0, 150);
            }

            neoPixel.showColors(pixArray);
        }
    }

    private static byte toByte(double x,
                               double minInput, double maxInput,
                               double minOutput, double maxOutput) {
        double range = maxInput - minInput;
        double frac = (x - minInput) / range;
        double output = (maxOutput - minOutput) * frac + minOutput;

        if (output > 255) {
            output = 255;
        } else if (output < 0) {
            output = 0;
        }

        return (byte) output;

    }
}