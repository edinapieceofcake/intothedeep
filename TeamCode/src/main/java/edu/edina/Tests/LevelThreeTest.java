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
    private DcMotorEx extension;

    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotorEx.class, "left_lift_motor");
        leftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        extension = hardwareMap.get(DcMotorEx.class, "extension_motor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "right_lift_motor");
        rightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        DcMotorEx[] motors = new DcMotorEx[]{leftMotor, rightMotor};
        for (DcMotorEx m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        waitForStart();

        actuallyRun(motors);
    }

    public  void actuallyRun(DcMotorEx[] motors) {
        int[] i = new int[2];

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

            if (gamepad1.dpad_down) {
                extension.setPower(-1);
            } else if (gamepad1.dpad_up) {
                extension.setPower(1);
            }

            armMotor.setPower(rightStick);

            telemetry.addData("left lift pos", i[0]);
            telemetry.addData("right lift pos", i[1]);
            telemetry.addData("arm pos", armMotor.getCurrentPosition());

            telemetry.update();
        }
    }
}
