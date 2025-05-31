package edu.edina.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Wrist Test", group = "Test")
public class WristTest extends LinearOpMode {
    public static double INTAKE_POSITION = 0.4;
    public static double SPECIMEN_POSITION = 1;
    public static double WALL_POSITION = (INTAKE_POSITION + SPECIMEN_POSITION) / 2;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo claw = hardwareMap.get(Servo.class, "wrist");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a)
                claw.setPosition(SPECIMEN_POSITION);
            else if (gamepad1.b)
                claw.setPosition(INTAKE_POSITION);
            else if (gamepad1.x)
                claw.setPosition(WALL_POSITION);
        }
    }
}
