package edu.edina.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import edu.edina.Libraries.Robot.RobotHardware;

@Disabled
public class OdometryAndIMU extends LinearOpMode {
    private double yawIMU;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        RobotHardware hw = new RobotHardware(this);

        yawIMU = 0.0;

        Pose2d pose = new Pose2d(0, 0, 0);

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
            /*
            hw.leftFrontDrive.setPower(leftFrontPower);
            hw.rightFrontDrive.setPower(rightFrontPower);
            hw.leftBackDrive.setPower(leftBackPower);
            hw.rightBackDrive.setPower(rightBackPower);
            */
            Twist2dDual<Time> t = hw.odometry.update();
            pose = pose.plus(t.value());

            yawIMU = hw.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            packet.put("x", pose.position.x);
            packet.put("y", pose.position.y);
            packet.put("heading", Math.toDegrees(pose.heading.toDouble()));
            packet.put("imu heading", yawIMU);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}