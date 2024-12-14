package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.edina.Libraries.Robot.INA219Device;

@Autonomous
public class PowerMonitorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        INA219Device monitor = hardwareMap.get(INA219Device.class, "power_monitor");

        waitForStart();

        while (opModeIsActive()) {
            byte[] buf = monitor.getReading();
            telemetry.addData("shunt voltage", "%x %x", (int) buf[0], (int) buf[1]);
            telemetry.update();
        }
    }
}
