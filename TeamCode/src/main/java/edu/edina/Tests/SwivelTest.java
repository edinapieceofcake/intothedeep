package edu.edina.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Swivel Test", group = "Test")
public class SwivelTest extends LinearOpMode {
    public static double HORIZONTAL_POS = 0;
    public static double VERTICAL_POS = 0.576;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo claw = hardwareMap.get(Servo.class, "swivel");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a)
                claw.setPosition(VERTICAL_POS);
            else if (gamepad1.b)
                claw.setPosition(HORIZONTAL_POS);
        }
    }
}

