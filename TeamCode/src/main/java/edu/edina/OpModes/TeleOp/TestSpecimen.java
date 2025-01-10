package edu.edina.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import edu.edina.Libraries.Robot.Drivetrain;

@TeleOp
public class TestSpecimen extends LinearOpMode {
    private Drivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {
        DistanceSensor sensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        drivetrain = new Drivetrain(this);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a)
                drivetrain.update(0.2, 0, 0);
            else
                drivetrain.update(0, 0, 0);

            telemetry.addData("distance", "%f", sensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}