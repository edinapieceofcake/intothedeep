package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import edu.edina.Libraries.Robot.RobotHardware;

@TeleOp
@Disabled
public class TeleOpProto extends LinearOpMode {
    private DcMotor liftMotor = null;

    @Override
    public void runOpMode() {
        DcMotorEx leftFrontDrive = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        DcMotorEx leftBackDrive = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        DcMotorEx rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        DcMotorEx rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double max;

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            if (gamepad1.dpad_up) {
                liftMotor.setPower(1);
            }
            if (gamepad1.dpad_down) {
                liftMotor.setPower(-1);
            }
            liftMotor.setPower(0.0);

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
        }
    }
}
