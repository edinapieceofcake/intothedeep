package edu.edina.Tests;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class DistanceSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Rev2mDistanceSensor leftDistance = hardwareMap.get(Rev2mDistanceSensor.class, "distance_left");
        Rev2mDistanceSensor rightDistance = hardwareMap.get(Rev2mDistanceSensor.class, "distance_right");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("left distance inches", "%.02f in", leftDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("right distance inches", "%.02f in", rightDistance.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
