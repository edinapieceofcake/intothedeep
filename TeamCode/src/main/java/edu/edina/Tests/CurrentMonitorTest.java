package edu.edina.Tests;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.nio.ByteBuffer;

import edu.edina.Libraries.Robot.INA219Device;
import edu.edina.Libraries.Robot.INA260Device;

@TeleOp
public class CurrentMonitorTest extends LinearOpMode {
    private FtcDashboard dashboard;
    private INA260Device dev;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        DataFile dataFile = new DataFile("current-monitor.csv");
        dataFile.println("Time,Reading");

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        dev = hardwareMap.get(INA260Device.class, "sensor_ina219");
        DcMotor motor = hardwareMap.get(DcMotor.class, "arm_motor");

        while (opModeInInit()) {
            readDevice();
            dataFile.showStatus(telemetry);
            telemetry.update();
        }

        ElapsedTime t = new ElapsedTime();

        while (opModeIsActive()) {
            telemetry.addLine("press A to run motor");

            if (gamepad1.a) {
                motor.setPower(0.2);
            } else {
                motor.setPower(0);
            }

            short reading = readDevice();
            dataFile.println(String.format("%f,%d", t.seconds(), reading));

            telemetry.update();
        }
    }

    private short readDevice() {
        byte[] buffer = dev.getReading();
        short reading = ByteBuffer.wrap(buffer).getShort();

        telemetry.addData("reading", reading);

        TelemetryPacket p = new TelemetryPacket();
        p.put("reading", reading);
        dashboard.sendTelemetryPacket(p);

        return reading;
    }
}
