package edu.edina.Tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import edu.edina.Libraries.Robot.Matrix2d;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.SparkFunOTOSCorrected;

@TeleOp
public class OdometryMultiTest extends LinearOpMode {
    SparkFunOTOS myOtos;
    double yawIMU;

    @Override
    public void runOpMode() {
        yawIMU = 0.0;

        Vector2d[] actualTestVecs = new Vector2d[]{
                new Vector2d(118.0 + 1.0 / 8.0, 0),
                new Vector2d(0, 118 + 1.0 / 8.0)
        };

        Vector2d[] otosTestVecs = new Vector2d[]{
                new Vector2d(-64.70, 0.89),
                new Vector2d(1.49, -77.64)
        };

        Matrix2d calOutputActual = Matrix2d.withColumns(actualTestVecs[0], actualTestVecs[1]);

        Matrix2d calOutputOtos = Matrix2d.withColumns(otosTestVecs[0], otosTestVecs[1]);

        Matrix2d otosCalMatrix = calOutputActual.mult(calOutputOtos.invert());

        myOtos = hardwareMap.get(SparkFunOTOSCorrected.class, "sensor_otos");
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        myOtos.setLinearScalar(0.867210119035);
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
            Vector2d otosVector = new Vector2d(posOTOS.x, posOTOS.y);
            otosVector = otosCalMatrix.transform(otosVector);

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

            Twist2dDual<Time> t = hw.odometry.update();
            pos3DW = pos3DW.plus(t.value());

            yawIMU = hw.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            telemetry.addData("imu heading", "", yawIMU);
            telemetry.addData("3DW x, y, heading", "%.2f, %.2f, %.2f", pos3DW.position.x, pos3DW.position.y, Math.toDegrees(pos3DW.heading.toDouble()));
            telemetry.addData("OTOS x, y, heading", "%.2f, %.2f, %.2f", posOTOS.x, posOTOS.y, posOTOS.h);
            telemetry.addLine();
            telemetry.addLine();
            telemetry.addData("calibrated otos reading", "%.2f, %.2f", otosVector.x, otosVector.y);
            telemetry.update();
        }
    }
}