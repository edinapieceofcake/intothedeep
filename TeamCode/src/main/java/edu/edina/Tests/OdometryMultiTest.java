package edu.edina.Tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import edu.edina.Libraries.Robot.RobotHardware;

@TeleOp
public class OdometryMultiTest extends LinearOpMode {
    SparkFunOTOS myOtos;
    double yawIMU;

    @Override
    public void runOpMode() {
        yawIMU = 0.0;

        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        myOtos.calibrateImu();

        myOtos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();

        RobotHardware hw = new RobotHardware(hardwareMap, telemetry);

        Pose2d pos3DW = new Pose2d(0, 0, 0);

        waitForStart();

        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D posOTOS = myOtos.getPosition();

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

            hw.leftFrontDrive.setPower(leftFrontPower);
            hw.rightFrontDrive.setPower(rightFrontPower);
            hw.leftBackDrive.setPower(leftBackPower);
            hw.rightBackDrive.setPower(rightBackPower);

            Twist2dDual<Time> t = hw.odometry.update();
            pos3DW = pos3DW.plus(t.value());

            yawIMU = hw.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            telemetry.addData("imu heading", "", yawIMU);
            telemetry.addData("3DW x, y, heading", "%.2f, %.2f, %.2f", pos3DW.position.x, pos3DW.position.y, Math.toDegrees(pos3DW.heading.toDouble()));
            telemetry.addData("OTOS x, y, heading", "%.2f, %.2f, %.2f", posOTOS.x, posOTOS.y, posOTOS.h);
            telemetry.update();
        }
    }
}