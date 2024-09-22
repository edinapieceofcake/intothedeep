package edu.edina.Tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Disabled
public class IMUTest extends LinearOpMode {
    private IMU imu;
    private YawPitchRollAngles angles;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {
            angles = imu.getRobotYawPitchRollAngles();
            double yaw = angles.getYaw(AngleUnit.DEGREES);

            telemetry.addData("heading", yaw);
            telemetry.update();
        }
    }
}
