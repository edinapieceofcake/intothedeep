package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous
public class TouchSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TouchSensor sensor=hardwareMap.get(TouchSensor.class, "arm_touch_front");
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("touching", sensor.getValue());
            telemetry.update();
        }
    }
}
