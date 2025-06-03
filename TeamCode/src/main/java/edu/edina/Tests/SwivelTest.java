package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import edu.edina.Libraries.Robot.Grabber;

@TeleOp(name = "Swivel Test", group = "Test")
public class SwivelTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo claw = hardwareMap.get(Servo.class, "swivel");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a)
                claw.setPosition(Grabber.VERTICAL_POS);
            else if (gamepad1.b)
                claw.setPosition(Grabber.HORIZONTAL_POS);
        }
    }
}

