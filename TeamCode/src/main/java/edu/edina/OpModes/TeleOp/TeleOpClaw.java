package edu.edina.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOpClaw extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo servo1 = hardwareMap.get(Servo.class, "servo");

        double pos = 0.160;
        double amount = 0;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                amount += 1;
            }

            if (amount > 0) {
                pos = 0; //orange = .160, black = 0
                servo1.setPosition(pos);
                sleep(100);
                pos = 0.27; //orange = .243, black = .4
                servo1.setPosition(pos);

                telemetry.addData("pos", pos);
                telemetry.addData("amount of tests", amount);

                amount -= 1;
            } else {
                telemetry.addLine("no more tests");
            }

            servo1.setPosition(pos);

            telemetry.update();
        }
    }
}