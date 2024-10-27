package edu.edina.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import edu.edina.Libraries.Robot.RobotHardware;

@TeleOp
public class TeleOpTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware hw = new RobotHardware(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                hw.liftMotorR.setPower(1);
                hw.liftMotorL.setPower(1);
            } else if (gamepad1.dpad_down) {
                hw.liftMotorR.setPower(-1);
                hw.liftMotorL.setPower(-1);
            } else{
                hw.liftMotorR.setPower(0);
                hw.liftMotorL.setPower(0);
            }

            if (gamepad1.right_bumper) {
                hw.slideServo.setDirection(DcMotorSimple.Direction.FORWARD);
                hw.slideServo.setPower(1);
            } else if (gamepad1.left_bumper) {
                hw.slideServo.setDirection(DcMotorSimple.Direction.REVERSE);
                hw.slideServo.setPower(1);
            } else {
                hw.slideServo.setPower(0);
            }

            if (gamepad1.left_trigger > 0.1) {
                hw.claw.setPosition(0);
            } else {
                hw.claw.setPosition(1);
            }

            telemetry.addData("claw pos", hw.claw.getPosition());
            telemetry.update();
        }
    }
}
