package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.Robot.LinearDrive;

@TeleOp
public class CalibrateLinearMechanism extends LinearOpMode {
    LinearDrive axialDrive;

    @Override
    public void runOpMode() {
        axialDrive = new LinearDrive(hardwareMap);

        waitForStart();

        telemetry.addLine("Press a to calibrate ppi");
        telemetry.update();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                calibratePpi();
            }
        }
    }

    private void calibratePpi() {
        while (opModeIsActive()) {
            telemetry.addLine("press up to drive forward");

            if (gamepad1.dpad_up) {
                axialDrive.setPower(0.3);
            } else {
                axialDrive.setPower(0.0);
            }

            telemetry.addData("encoder (raw)", axialDrive.getPosition(true));
            telemetry.addData("encoder (inch)", axialDrive.getPosition(false));
            telemetry.update();
        }
    }
}