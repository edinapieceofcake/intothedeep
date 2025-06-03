package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import edu.edina.Libraries.Robot.Grabber;

@TeleOp(name = "Claw Test", group = "Test")
public class ClawTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo claw = hardwareMap.get(Servo.class, "claw_bottom");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a)
                claw.setPosition(Grabber.OPEN_POS);
            else if (gamepad1.b)
                claw.setPosition(Grabber.CLOSED_POS);
        }
    }
}

