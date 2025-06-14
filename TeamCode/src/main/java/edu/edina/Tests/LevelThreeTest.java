package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Level 3 Test", group = "Test")
public class LevelThreeTest extends LinearOpMode {
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private DcMotorEx armMotor;

    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotorEx.class, "left_lift_motor");
        leftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightMotor = hardwareMap.get(DcMotorEx.class, "right_lift_motor");
        rightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotorEx[] motors = new DcMotorEx[]{leftMotor, rightMotor};
        for (DcMotorEx m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        int[] i = new int[2];

        waitForStart();

        while (opModeIsActive()) {
            double rightStick = Math.signum(-gamepad1.right_stick_y) * Math.pow(-gamepad1.right_stick_y, 2);
            double leftStick = Math.signum(-gamepad1.left_stick_y) * Math.pow(-gamepad1.left_stick_y, 2);
            telemetry.addData("right stick", rightStick);
            telemetry.addData("left stick", leftStick);
            telemetry.addLine("--------");

            int j = 0;
            for (DcMotorEx m : motors) {
                m.setPower(leftStick);
                i[j] = m.getCurrentPosition();
                j++;
            }

            if (gamepad1.right_trigger > 0.7) {
                armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            } else {
                armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            armMotor.setPower(rightStick);

            telemetry.addData("left lift pos", i[0]);
            telemetry.addData("right lift pos", i[1]);
            telemetry.addData("arm pos", armMotor.getCurrentPosition());

            telemetry.update();
        }
    }
}
