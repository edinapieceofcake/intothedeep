package edu.edina.Tests;

import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import android.app.Activity;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import edu.edina.Libraries.Robot.Light;
import edu.edina.Libraries.Robot.SampleColor;

@TeleOp
public class ColorSensorLightTest extends LinearOpMode {
    private Light light;
    private NormalizedColorSensor colorSensor;
    private View relativeLayout;

    @Override
    public void runOpMode() throws InterruptedException {
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        light = new Light(hardwareMap);

        try {
            runSample();
        } finally {
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
        }
    }

    protected void runSample() {
        float gain = 2;

        final float[] hsvValues = new float[3];

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.\n");
            telemetry.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value\n");

            if (gamepad1.a) {
                gain += 0.005;
            } else if (gamepad1.b && gain > 1) {
                gain -= 0.005;
            }

            telemetry.addData("gain", gain);
            // Tell the sensor our desired gain value (normally you would do this during initialization,
            // not during the loop)
            colorSensor.setGain(gain);

            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);


            telemetry.addData("red:", "%.3f", colors.red);
            telemetry.addData("green:", "%.3f", colors.green);
            telemetry.addData("blue:", "%.3f", colors.blue);
            telemetry.addLine();
            telemetry.addData("Hue", "%.3f", hsvValues[0]);
            telemetry.addData("Saturation", "%.3f", hsvValues[1]);
            telemetry.addData("Value", "%.3f", hsvValues[2]);

            if ((hsvValues[0] < 10 || hsvValues[0] > 340) && hsvValues[1] > 0.350) {
                setColor(SampleColor.RED);
            } else if ((hsvValues[0] < 70 && hsvValues[0] > 10) && hsvValues[1] > 0.400) {
                setColor(SampleColor.YELLOW);
            } else if ((hsvValues[0] < 230 && hsvValues[0] > 175) && hsvValues[1] > 0.300) {
                setColor(SampleColor.BLUE);
            } else {
                setColor(SampleColor.NOTHING);
            }

//            if (gamepad1.y)
//                setColor(SampleColor.YELLOW);
//            else if (gamepad1.b)
//                setColor(SampleColor.RED);
//            else if (gamepad1.x)
//                setColor(SampleColor.BLUE);
//            else
//                setColor(SampleColor.NOTHING);

            light.update();

            telemetry.update();

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
                }
            });
        }
    }

    public void setColor(SampleColor sampleColor) {
        light.setSampleColor(sampleColor);
    }
}
