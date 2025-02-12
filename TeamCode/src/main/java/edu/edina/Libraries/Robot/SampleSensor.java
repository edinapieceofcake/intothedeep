package edu.edina.Libraries.Robot;

import android.annotation.SuppressLint;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SampleSensor {
    private NormalizedColorSensor colorSensor;
    private float[] hsvValues;
    private SampleColor sampleColor;

    public SampleSensor(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        hsvValues = new float[3];
    }

    public SampleColor detectSampleColor() {
        colorSensor.setGain(2);

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        if ((hsvValues[0] < 10 || hsvValues[0] > 340) && hsvValues[1] > 0.350) {
            sampleColor = SampleColor.RED;
        } else if ((hsvValues[0] < 70 && hsvValues[0] > 10) && hsvValues[1] > 0.400) {
            sampleColor = SampleColor.YELLOW;
        } else if ((hsvValues[0] < 230 && hsvValues[0] > 175) && hsvValues[1] > 0.300) {
            sampleColor = SampleColor.BLUE;
        } else {
            sampleColor = SampleColor.NOTHING;
        }

        return sampleColor;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("sample sensor", "HSV: %f, %f, %f", hsvValues[0], hsvValues[1], hsvValues[2]);
        telemetry.addData("sample sensor", "last color: %s", sampleColor);
    }
}
