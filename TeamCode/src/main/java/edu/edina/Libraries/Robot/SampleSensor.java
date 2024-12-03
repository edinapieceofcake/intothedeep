package edu.edina.Libraries.Robot;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class SampleSensor {
    private NormalizedColorSensor colorSensor;

    public SampleSensor(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
    }

    public SampleColor detectSampleColor() {
        colorSensor.setGain(2);

        final float[] hsvValues = new float[3];

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        if ((hsvValues[0] < 10 || hsvValues[0] > 340) && hsvValues[1] > 0.350) {
            return SampleColor.RED;
        } else if ((hsvValues[0] < 70 && hsvValues[0] > 10) && hsvValues[1] > 0.400) {
            return SampleColor.YELLOW;
        } else if ((hsvValues[0] < 230 && hsvValues[0] > 175) && hsvValues[1] > 0.300) {
            return SampleColor.BLUE;
        } else {
            return SampleColor.NOTHING;
        }
    }
}
