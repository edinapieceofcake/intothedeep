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

import edu.edina.Libraries.RoadRunner.Localizer;
import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.RoadRunner.ThreeDeadWheelLocalizer;
import edu.edina.Libraries.Robot.ArmLift;
import edu.edina.Libraries.Robot.GamePadClick;
import edu.edina.Libraries.Robot.RobotHardware;

@TeleOp
public class HardwareTestMode extends LinearOpMode {
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

    private RobotHardware hw;
    private GamePadClick click1;

    // make field

    @Override
    public void runOpMode() throws InterruptedException {
        click1 = new GamePadClick(gamepad1);
        hw = new RobotHardware(this);


        int testIndex = 0;

        waitForStart();

        while (testIsActive()) {
            if (click1.dpad_up) {
                if (testIndex < tests.length - 1)
                    testIndex++;
            }

            if (click1.dpad_down) {
                if (testIndex > 0)
                    testIndex--;
            }

            if (click1.b) {
                ITestMode t = tests[testIndex];
                t.runTest();
            } else {
                telemetry.addData("test", tests[testIndex].getName());
                telemetry.update();
            }
        }
    }

    private boolean testIsActive() {
        click1.read();
        return opModeIsActive();
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
            ThreeDeadWheelLocalizer odometry = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick);

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

                hw.leftFrontDrive.setPower(leftFrontPower * 0.3);
                hw.rightFrontDrive.setPower(rightFrontPower * 0.3);
                hw.leftBackDrive.setPower(leftBackPower * 0.3);
                hw.rightBackDrive.setPower(rightBackPower * 0.3);

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

                hw.leftFrontDrive.setPower(0);
                hw.rightFrontDrive.setPower(0);
                hw.leftBackDrive.setPower(0);
                hw.rightBackDrive.setPower(0);

                if (gamepad1.dpad_left)
                    hw.leftFrontDrive.setPower(0.7);
                if (gamepad1.dpad_up)
                    hw.rightFrontDrive.setPower(0.7);
                if (gamepad1.dpad_down)
                    hw.leftBackDrive.setPower(0.7);
                if (gamepad1.dpad_right)
                    hw.rightBackDrive.setPower(0.7);

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
            Servo wrist = hw.wristLeft;
            Servo claw = hw.claw;

            double pos = 0;

            telemetry.clearAll();

            while (opModeIsActive()) {
                if (gamepad1.left_bumper) {
                    pos = 0;
                }
                if (gamepad1.right_bumper) {
                    pos = 1;
                }

                telemetry.addData("pos", pos);
                telemetry.update();

                wrist.setPosition(pos);
                claw.setPosition(pos);
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
            ArmLift lift = new ArmLift(hw);

            while (opModeIsActive()) {
                if (gamepad1.dpad_up) {
                    lift.setPower(0.3);
                } else if (gamepad1.dpad_down) {
                    lift.setPower(-0.3);
                } else {
                    lift.setPower(0.0);
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
            CRServo slide = hw.slideServo;

            double pow = 0.0;

            waitForStart();

            while (testIsActive()) {
                if (gamepad1.dpad_up) {
                    pow += 0.01;
                }
                if (gamepad1.dpad_down) {
                    pow -= 0.01;
                }
                if (click1.a) {
                    pow *= -1.0;
                }

                slide.setPower(pow);
            }
        }
    }

    private class LiftMotorTest implements ITestMode {
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
        public String getName() {
            return "DriveMotorTest";
        }

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
            double leftBackPower = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            double leftFrontPower = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            double rightBackPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            double rightFrontPower = gamepad1.b ? 1.0 : 0.0;  // B gamepad

            // Move the robot.
            hw.leftFrontDrive.setPower(leftFrontPower);
            hw.rightFrontDrive.setPower(rightFrontPower);
            hw.leftBackDrive.setPower(leftBackPower);
            hw.rightBackDrive.setPower(rightBackPower);
        }
    }
}