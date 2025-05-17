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
import edu.edina.Libraries.Robot.PowerReading;

@TeleOp
public class CurrentMonitorTest extends LinearOpMode {
    private FtcDashboard dashboard;
    private INA260Device dev;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        DataFile dataFile = new DataFile("current-monitor.csv");
        dataFile.println("Time,Current,Voltage");

        dev = hardwareMap.get(INA260Device.class, "sensor_ina260");
        DcMotor motor = hardwareMap.get(DcMotor.class, "arm_motor");

        while (opModeInInit()) {
            PowerReading r = dev.getReading();
            dataFile.showStatus(telemetry);
            telemetry.addData("current", "%.1fA", r.getCurrent());
            telemetry.addData("voltage", "%.1fv", r.getVoltage());
            telemetry.update();
        }

        ElapsedTime t = new ElapsedTime();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                motor.setPower(0.8);
            } else {
                motor.setPower(0);
            }

            PowerReading r = dev.getReading();

            dataFile.println(String.format("%f,%f,%f", t.seconds(), r.getCurrent(), r.getVoltage()));

            dataFile.showStatus(telemetry);
            telemetry.addData("current", "%.1fA", r.getCurrent());
            telemetry.addData("voltage", "%.1fv", r.getVoltage());
            telemetry.addLine("press A to run motor");
            telemetry.update();
        }
    }
}
