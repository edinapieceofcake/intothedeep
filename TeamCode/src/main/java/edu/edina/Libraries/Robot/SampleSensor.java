package edu.edina.Libraries.Robot;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SampleSensor {
    private NormalizedColorSensor colorSensor;
    private NormalizedRGBA rgb;
    private float[] hsvValues;
    private SampleColor sampleColor;

    public SampleSensor(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        hsvValues = new float[3];
        colorSensor.setGain(2);
    }

    public SampleColor detectSampleColor() {
        rgb = colorSensor.getNormalizedColors();
        Color.colorToHSV(rgb.toColor(), hsvValues);

        if ((hsvValues[0] < 60 && hsvValues[0] >= 0) && hsvValues[1] > 0.55) {
            sampleColor = SampleColor.RED;
        } else if ((hsvValues[0] < 120 && hsvValues[0] >= 60) && hsvValues[1] > 0.55 && hsvValues[2] > 0.006) {
            sampleColor = SampleColor.YELLOW;
        } else if ((hsvValues[0] < 255 && hsvValues[0] > 175) && hsvValues[1] > 0.55) {
            sampleColor = SampleColor.BLUE;
        } else {
            sampleColor = SampleColor.NOTHING;
        }

        return sampleColor;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("color sensor", "r=%.3f, g=%.3f, b=%.3f", rgb.red, rgb.green, rgb.blue);
        telemetry.addData("color sensor", "h=%.0f, s=%.3f, v=%.3f", hsvValues[0], hsvValues[1], hsvValues[2]);
        telemetry.addData("sample detected", "%s", sampleColor);
    }
}
