package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.Robot.Arm2;

@TeleOp
public class ArmTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Arm2.Mechanism arm = new Arm2.Mechanism(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double pos = arm.getPosition(false);

            telemetry.addLine("press up/down to move arm");
            telemetry.addData("position", "%.1f", pos);
            telemetry.update();

            if (pos > 180)
                arm.setPower(0);
            else if (gamepad1.dpad_up)
                arm.setPower(1);
            else if (gamepad1.dpad_down)
                arm.setPower(-1);
            else
                arm.setPower(0);
        }
    }
}
