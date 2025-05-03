package edu.edina.Tests;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.Robot.INA219Device;

@TeleOp
public class CurrentMonitorTest extends LinearOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        INA219Device dev = hardwareMap.get(INA219Device.class, "sensor_ina219");
        waitForStart();
        while (opModeIsActive()) {
            byte[] buffer = dev.getReading();
            for (int i = 0; i < buffer.length; i++) {
                telemetry.addData(String.format("b%d", i), "%x", buffer[i]);
            }

            telemetry.update();
        }
    }
}
