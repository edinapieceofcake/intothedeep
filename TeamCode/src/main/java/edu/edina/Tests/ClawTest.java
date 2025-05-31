package edu.edina.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Claw Test", group = "Test")
public class ClawTest extends LinearOpMode {
    public static double OPEN_POS = 0.4;
    public static double CLOSED_POS = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo claw = hardwareMap.get(Servo.class, "claw_bottom");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a)
                claw.setPosition(OPEN_POS);
            else if (gamepad1.b)
                claw.setPosition(CLOSED_POS);
        }
    }
}
