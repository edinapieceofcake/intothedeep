package edu.edina.Libraries.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Light {
    private byte[] pixArray;
    private int numPixel;
    private NeoPixelDriverDevice neoPixel;
    private ElapsedTime t;
    private SampleColor sampleColor;
    private SampleSensor sampleSensor;

    private final double LIGHT_MULT;

    public Light(HardwareMap hardwareMap, SampleSensor sampleSensor) {
        neoPixel = hardwareMap.get(NeoPixelDriverDevice.class, "neopixel_driver");
        numPixel = NeoPixelDriverDevice.NUM_PIXELS;
        pixArray = new byte[NeoPixelDriverDevice.NUM_BYTES];
        t = new ElapsedTime();
        this.sampleSensor = sampleSensor;
        LIGHT_MULT = 0.07;
    }

    public void update(boolean wave, boolean sample) {
        setSampleColor();
        updateWaveEffect(wave);
        showSample(sample);
        neoPixel.showColors(pixArray);
    }

    private void updateWaveEffect(boolean on) {
        if (on) {
            double x;

            double blueTime = Math.sin(t.seconds() * 1.5);
            double a = 1 / 40.0;
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

            //pixel before and after white
            int firstPixel = numPixel / 2;

            for (int n = 0; n < 3; n++) {
                pixArray[(byte) ((firstPixel - 3) * 3 + n)] = (byte) (255 * LIGHT_MULT);
                pixArray[(byte) ((firstPixel + 2) * 3 + n)] = (byte) (255 * LIGHT_MULT);
            }
        }
    }

    public void setSampleColor() {
        sampleColor = sampleSensor.detectSampleColor();
    }

    public void showSample(boolean on) {
        if (on) {
            byte r, g, b;
            if (sampleColor == SampleColor.RED) {
                r = (byte) 30;
                g = (byte) 0;
                b = (byte) 0;
            } else if (sampleColor == SampleColor.BLUE) {
                r = (byte) 0;
                g = (byte) 0;
                b = (byte) 30;
            } else if (sampleColor == SampleColor.YELLOW) {
                r = (byte) 30;
                g = (byte) 30;
                b = (byte) 0;
            } else {
                r = (byte) 30;
                g = (byte) 30;
                b = (byte) 30;
            }

            for (int n = 0; n < numPixel; n++) {
                int greenByte = n * 3;
                int redByte = n * 3 + 1;
                int blueByte = n * 3 + 2;

                pixArray[greenByte] = g;
                pixArray[redByte] = r;
                pixArray[blueByte] = b;
            }
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