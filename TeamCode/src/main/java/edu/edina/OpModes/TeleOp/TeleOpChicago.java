package edu.edina.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import edu.edina.Libraries.Robot.RobotHardwareChicago;
import edu.edina.Libraries.Robot.Speedometer;
import edu.edina.Tests.LevelThreeTest;

@TeleOp(name = "TeleOp Main \uD83C\uDF82", group = "Main")
public class TeleOpChicago extends LinearOpMode {
    private boolean hang = false;
    private Gamepad currentGamepad1, previousGamepad1, currentGamepad2, previousGamepad2;
    private int cycleNum;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Speedometer s = new Speedometer(20);

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        RobotHardwareChicago hw = new RobotHardwareChicago(hardwareMap);

        while (opModeInInit()) {
            hw.initUpdate(telemetry);
            telemetry.update();
        }

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            hw.update(telemetry);
            hw.drive(gamepad1, gamepad2);

            if (previousGamepad1.back && !currentGamepad1.back) {
                hang = !hang;
            }

            if (hang) {
                ascentMode();
            }

            if (currentGamepad2.back) {
                hw.brake();
                continue;
            }
            if (previousGamepad2.back && !currentGamepad2.back) {
                hw.calibrateIMU();
            }
            if (currentGamepad1.a && !previousGamepad1.a) {
                hw.toggleClaw();
            }
            if (currentGamepad1.b && !previousGamepad1.b) {
                hw.wallMode();
            }

            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                hw.highSpecimen();
            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper && currentGamepad1.right_trigger >= 0.7) {
                hw.lowSpecimen();
            }
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                hw.highBasketMode();
            }
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper && currentGamepad1.right_trigger >= 0.7) {
                hw.lowBasketMode();
            }
            if (hw.armOverSub()) {
                if (currentGamepad2.right_trigger >= 0.7) {
                    hw.intake();
                }

                if (currentGamepad2.left_bumper && previousGamepad2.left_bumper && currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                    hw.halfSwivel();
                } else if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper && currentGamepad2.right_bumper && previousGamepad2.right_bumper) {
                    hw.halfSwivel();
                } else if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                    hw.perpendicularSwivel();
                } else if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                    hw.horizontalSwivel();
                }
                hw.extend(-gamepad2.right_stick_y);
            }

            if (currentGamepad2.x && !previousGamepad2.x) {
                hw.subMode();
            }

            if (gamepad2.dpad_up && !previousGamepad2.dpad_up) {
                hw.getRobotState().adjustArmCalibration(1);
            } else if (gamepad2.dpad_down && !previousGamepad2.dpad_down) {
                hw.getRobotState().adjustArmCalibration(-1);
            }

            s.sample(cycleNum);
            cycleNum++;

            telemetry.addData("cycleSpeed", "%.1f", 1000.0 / s.getSpeed());

            telemetry.update();
        }
    }

    private void ascentMode() {

        DcMotorEx leftMotor;
        DcMotorEx rightMotor;
        DcMotorEx armMotor;
        DcMotorEx extension;

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
