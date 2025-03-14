package edu.edina.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import edu.edina.Libraries.RoadRunner.Localizer;
import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.RoadRunner.ThreeDeadWheelLocalizer;
import edu.edina.Libraries.Robot.Drivetrain;

@Config
@TeleOp
public class OdoCompareTest extends LinearOpMode {
    public static double angularScalar = 0.99173554;
    public static double linearScalar = 1.04712042;

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain dt = new Drivetrain(this);

        SparkFunOTOS otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        if (!otos.calibrateImu()) {
            telemetry.addLine("failed to calibrate OTOS IMU");
        }

        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularScalar(angularScalar);
        otos.setLinearScalar(linearScalar);
        otos.setOffset(new SparkFunOTOS.Pose2D(6.4, 1.6, 180));

        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);
        telemetry.addData("OTOS version", "hw: %d.%d, fw: %d.%d",
                hwVersion.major, hwVersion.minor,
                fwVersion.major, fwVersion.minor);

        Localizer localizer = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick);
        telemetry.update();

        waitForStart();

        Pose2d deadwheelPose = new Pose2d(new Vector2d(0, 0), 0);
        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));

        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D opticalPose = otos.getPosition();
            opticalPose = new SparkFunOTOS.Pose2D(opticalPose.x, opticalPose.y, opticalPose.h);

            Twist2dDual<Time> twist = localizer.update();
            deadwheelPose = deadwheelPose.plus(twist.value());

            dt.update();

            telemetry.addData("optical pose", "(%.1f, %.1f) %f",
                    opticalPose.x, opticalPose.y, opticalPose.h);
            telemetry.addData("deadwheel pose", "(%.1f, %.1f) %f",
                    deadwheelPose.position.x, deadwheelPose.position.y,
                    Math.toDegrees(deadwheelPose.heading.toDouble()));

            telemetry.update();
        }
    }
}
