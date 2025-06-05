package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import edu.edina.Libraries.Robot.Grabber;

@TeleOp(name = "Wrist Test", group = "Test")
public class WristTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo claw = hardwareMap.get(Servo.class, "wrist");

        waitForStart();

        telemetry.addData("gamepad1.a", "SPECIMEN_POSITION");
        telemetry.addData("gamepad1.b", "INTAKE_POSITION");
        telemetry.addData("gamepad1.x", "WALL_POSITION");
        telemetry.update();

        while (opModeIsActive()) {
            if (gamepad1.a)
                claw.setPosition(Grabber. SPECIMEN_POSITION);
            else if (gamepad1.b)
                claw.setPosition(Grabber.INTAKE_POSITION);
            else if (gamepad1.x)
                claw.setPosition(Grabber.WALL_POSITION);
        }
    }
}
