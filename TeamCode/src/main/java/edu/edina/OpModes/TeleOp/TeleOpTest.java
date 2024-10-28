package edu.edina.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import edu.edina.Libraries.Robot.RobotHardware;

@TeleOp
public class TeleOpTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware hw = new RobotHardware(hardwareMap);

        waitForStart();

        double pos = 0;

        while (opModeIsActive()) {
            pos = hw.claw.getPosition();

            if (gamepad1.dpad_up) {
                hw.liftMotorR.setPower(1);
                hw.liftMotorL.setPower(1);
            } else if (gamepad1.dpad_down) {
                hw.liftMotorR.setPower(-1);
                hw.liftMotorL.setPower(-1);
            } else {
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
                hw.claw.setPosition(pos + 0.001);
            } else if (gamepad1.right_trigger > 0.1) {
                hw.claw.setPosition(pos - 0.001);
            }

            if (gamepad1.a) {
                hw.armMotor.setPower(1.0);
            } else if (gamepad1.b) {
                hw.armMotor.setPower(-1.0);
            } else {
                hw.armMotor.setPower(0.0);
            }

            telemetry.addData("claw pos", hw.claw.getPosition());
            telemetry.addData("left lift", hw.getLeftLift());
            telemetry.addData("right lift", hw.getRightLift());
            telemetry.addData("arm motor", hw.getArmMotor());
            telemetry.addData("slide enc", hw.getSlideVoltage());
            telemetry.update();
        }
    }
}
