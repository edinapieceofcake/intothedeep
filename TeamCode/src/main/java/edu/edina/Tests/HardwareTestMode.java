package edu.edina.Tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.RoadRunner.ThreeDeadWheelLocalizer;

@TeleOp
public class HardwareTestMode extends LinearOpMode {
    private ThreeDeadWheelLocalizer odometry;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor liftMotor = null;
    private Servo servo1 = null;

    // make field

    @Override
    public void runOpMode() {
        try {
            odometry = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick);
        } catch (Exception e) {
            //ignore
        }

        try {
            leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
            leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
            rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            //ignore
        }

        try {
            liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            //ignore
        }

        try {
            servo1 = hardwareMap.get(Servo.class, "servo");
        } catch (Exception e) {
            //ignore
        }

        boolean previousDPadUp = false;
        boolean previousDPadDown = false;

        // construct the tests
        ITestMode[] tests = new ITestMode[]{
                new ImuTest(),
                new DeadwheelTest(),
                new DriveMotorsTest(),
                new ServoTest(),
                new LiftMotorTest()
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

        @Override
        public String getName() {
            return "IMU";
        }

        @Override
        public void runTest() {
            imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            imu.initialize(parameters);

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

    private class ServoTest implements ITestMode {
        @Override
        public String getName() {
            return "Servo Test";
        }

        @Override
        public void runTest() {
            double pos = 0.0;

            telemetry.clearAll();

            while (testIsActive()) {
                if (gamepad1.dpad_up) {
                    pos = Math.min(pos + 0.1, 1);
                }
                if (gamepad1.dpad_down) {
                    pos = Math.max(pos - 0.1, 0);
                }

                telemetry.addData("pos", pos);
                telemetry.update();

                servo1.setPosition(pos);
            }
        }
    }

    private class LiftMotorTest implements ITestMode {
        @Override
        public String getName() {
            return "Lift Test";
        }

        public void runTest() {
            while (testIsActive()) {
                while (gamepad1.dpad_up) {
                    liftMotor.setPower(0.2);
                }
                while (gamepad1.dpad_down) {
                    liftMotor.setPower(-0.2);
                }
            }

            liftMotor.setPower(0);
        }
    }
}