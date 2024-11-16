package edu.edina.Libraries.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Light {
    private byte[] pixArray;
    private int numPixel;
    private NeoPixelDriverDevice neoPixel;
    private ElapsedTime t;


    public Light(HardwareMap hardwareMap) {
        neoPixel = hardwareMap.get(NeoPixelDriverDevice.class, "neopixel_driver");
        numPixel = NeoPixelDriverDevice.NUM_PIXELS;
        pixArray = new byte[NeoPixelDriverDevice.NUM_BYTES];
        t = new ElapsedTime();
    }

    public void update() {

    }
}