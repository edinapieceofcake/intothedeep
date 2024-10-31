package edu.edina.Tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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
    private Servo servo3 = null;
    private CRServo servo2 = null;
    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;

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
            liftMotor = hardwareMap.get(DcMotor.class, "motor0");
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            //ignore
        }

        try {
            servo1 = hardwareMap.get(Servo.class, "wrist_left");
            servo3 = hardwareMap.get(Servo.class, "wrist_right");
        } catch (Exception e) {
            //ignore
        }

        try {
            servo2 = hardwareMap.get(CRServo.class, "servo");
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
                new MotorTest(),
                new CRServoTest(),
                new DriveMotorTest()
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
            double pos = 0;

            telemetry.clearAll();

            while (testIsActive()) {
                if (gamepad1.left_bumper) {
                    pos = 0;
                }
                if (gamepad1.right_bumper) {
                    pos = 1;
                }

                telemetry.addData("pos", pos);
                telemetry.update();

                servo1.setPosition(pos);
                servo3.setPosition(pos);
            }
        }
    }

    private class MotorTest implements ITestMode {
        @Override
        public String getName() {
            return "Motor Test";
        }

        @Override
        public void runTest() {
            while (testIsActive()) {
                if (gamepad1.dpad_up) {
                    liftMotor.setPower(1);
                } else if (gamepad1.dpad_down) {
                    liftMotor.setPower(-1);
                } else {
                    liftMotor.setPower(0.0);
                }
            }
        }
    }

    private class CRServoTest implements ITestMode {
        @Override
        public String getName() {
            return "Continuous Servo Test";
        }

        @Override
        public void runTest() {
            double pow = 0.0;

            waitForStart();

            while (opModeIsActive()) {
                if (gamepad1.dpad_up) {
                    pow += 0.01;
                }
                if (gamepad1.dpad_down) {
                    pow -= 0.01;
                }
                if (gamepad1.a) {
                    pow *= -1.0;
                }

                servo2.setPower(pow);
            }
        }
    }

    private class LiftMotorTest implements ITestMode{
        @Override
        public String getName() {
            return "Lift Motor Test";
        }

        @Override
        public void runTest() {

        }
    }

    private class ArmMotorTest implements ITestMode {
        @Override
        public String getName() {
            return "ArmMotorTest";
        }

        @Override
        public void runTest() {

        }
    }

    private class DriveMotorTest implements ITestMode {
        @Override
        public String getName() {return "DriveMotorTest";}

        @Override
        public void runTest() {
            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.
            leftBackPower = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftFrontPower = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightBackPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightFrontPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad

            // Move the robot.
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
        }
    }
}