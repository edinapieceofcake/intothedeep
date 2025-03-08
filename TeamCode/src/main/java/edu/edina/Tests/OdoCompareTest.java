package edu.edina.Tests;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.edina.Libraries.Robot.RobotHardware;

public class OdoCompareTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);

        SparkFunOTOS otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

//        hw.getOdometry();
    }
}
