package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;

@TeleOp
public class AnalogTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        AnalogInput test = hardwareMap.get(AnalogInput.class, "test");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("voltage", test.getVoltage());
            telemetry.addData("max voltage", test.getMaxVoltage());
            telemetry.addData("version", test.getVersion());
            telemetry.addData("connection info", test.getConnectionInfo());
            telemetry.addData("device name", test.getDeviceName());
            telemetry.addData("manufacturer", test.getManufacturer());
            telemetry.update();
        }
    }
}
