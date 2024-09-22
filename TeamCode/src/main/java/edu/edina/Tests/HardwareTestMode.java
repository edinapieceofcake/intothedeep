package edu.edina.Tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import roadrunner.MecanumDrive;
import roadrunner.ThreeDeadWheelLocalizer;

@TeleOp
public class HardwareTestMode extends LinearOpMode {
    private ThreeDeadWheelLocalizer odometry;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // make field

    @Override
    public void runOpMode() {
        odometry = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick);

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        boolean previousDPadUp = false;
        boolean previousDPadDown = false;

        // construct the tests
        ITestMode[] tests = new ITestMode[]{
                new ImuTest(),
                new DeadwheelTest(),
                new DriveMotorsTest(),
        };

        int testIndex = 0;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                if (!previousDPadUp) {
                    if (testIndex < tests.length - 1)
                        testIndex++;
                    previousDPadUp = true;
                    continue;
                }
            } else {
                previousDPadUp = false;
            }

            if (gamepad1.dpad_down) {
                if (!previousDPadDown) {
                    if (testIndex > 0)
                        testIndex--;

                    previousDPadDown = gamepad1.dpad_down;
                    continue;
                }
            } else {
                previousDPadDown = false;
            }

            if (gamepad1.b) {
                ITestMode t = tests[testIndex];
                t.runTest();
            } else {
                telemetry.addData("test", tests[testIndex].getName());
                telemetry.update();
            }
        }
    }

    private boolean testIsActive() {
        if (gamepad1.a) {
            telemetry.clearAll();
            telemetry.setDisplayFormat(Telemetry.DisplayFormat.CLASSIC);
            return false;
        } else {
            return opModeIsActive();
        }
    }

    private interface ITestMode {
        String getName();

        void runTest();
    }

    private class ImuTest implements ITestMode {
        private IMU imu;

        private ImuTest() {
            imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            imu.initialize(parameters);
        }

        @Override
        public String getName() {
            return "IMU";
        }

        @Override
        public void runTest() {
            while (testIsActive()) {
                YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
                double yaw = angles.getYaw(AngleUnit.DEGREES);

                telemetry.addData("heading", yaw);
                telemetry.update();
            }
        }
    }

    private class DeadwheelTest implements ITestMode {
        @Override
        public String getName() {
            return "Deadwheel Test";
        }

        @Override
        public void runTest() {
            int basePosPar0 = 0;
            int basePosPar1 = 0;
            int basePosPerp = 0;

            while (testIsActive()) {
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

                leftFrontDrive.setPower(leftFrontPower * 0.3);
                rightFrontDrive.setPower(rightFrontPower * 0.3);
                leftBackDrive.setPower(leftBackPower * 0.3);
                rightBackDrive.setPower(rightBackPower * 0.3);

                telemetry.addData("Left", odometry.par0.getPositionAndVelocity().position - basePosPar0);
                telemetry.addData("Right", odometry.par1.getPositionAndVelocity().position - basePosPar1);
                telemetry.addData("Back", odometry.perp.getPositionAndVelocity().position - basePosPerp);

                telemetry.addLine();
                telemetry.addLine("Press Y to reset values");
                if (gamepad1.y) {
                    basePosPar0 = odometry.par0.getPositionAndVelocity().position;
                    basePosPar1 = odometry.par1.getPositionAndVelocity().position;
                    basePosPerp = odometry.perp.getPositionAndVelocity().position;
                }

                telemetry.update();
            }
        }
    }

    private class DriveMotorsTest implements ITestMode {
        @Override
        public String getName() {
            return "Drive Motors";
        }

        @Override
        public void runTest() {
            telemetry.clearAll();
            telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

            while (testIsActive()) {
                telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;Xbox Button - Motor</font>");
                telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;&larr;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Left</font>");
                telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;&uarr;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Right</font>");
                telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;&rarr;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Right</font>");
                telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;&darr;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Left</font>");

                if (gamepad1.dpad_left)
                    leftFrontDrive.setPower(0.7);
                if (gamepad1.dpad_up)
                    rightFrontDrive.setPower(0.7);
                if (gamepad1.dpad_down)
                    leftBackDrive.setPower(0.7);
                if (gamepad1.dpad_right)
                    rightBackDrive.setPower(0.7);

                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);

                telemetry.update();
            }
        }
    }
}