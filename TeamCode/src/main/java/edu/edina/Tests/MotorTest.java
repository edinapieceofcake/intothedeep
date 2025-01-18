package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Disabled
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        String name = "slide_motor";

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);

        waitForStart();

        while (opModeIsActive()) {
            double power = -gamepad1.left_stick_y;
            motor.setPower(power);
            telemetry.addLine(String.format("testing motor %s with left y-stick", name));
            telemetry.addData("power", power);
            telemetry.update();
        }
    }
}
