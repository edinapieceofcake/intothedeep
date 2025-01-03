package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class EncoderTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx armExtension = hardwareMap.get(DcMotorEx.class, "extension_motor");
        DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "arm_motor");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("arm", arm.getCurrentPosition());
            telemetry.addData("extension", armExtension.getCurrentPosition());
            telemetry.update();
        }
    }
}