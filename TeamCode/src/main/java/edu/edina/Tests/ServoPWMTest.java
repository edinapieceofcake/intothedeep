package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
@Disabled
public class ServoPWMTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ServoImplEx s = hardwareMap.get(ServoImplEx.class, "wrist");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                s.setPosition(0);
            }
            if (gamepad1.dpad_down) {
                s.setPosition(1);
            }

            if (gamepad1.a) {
                s.setPwmEnable();
            }

            if (gamepad1.b) {
                s.setPwmDisable();
            }
        }
    }
}
