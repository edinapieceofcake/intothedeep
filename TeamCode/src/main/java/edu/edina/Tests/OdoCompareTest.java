package edu.edina.Tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.RoadRunner.Localizer;
import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.RoadRunner.ThreeDeadWheelLocalizer;
import edu.edina.Libraries.Robot.Drivetrain;
import edu.edina.Libraries.Robot.OpticalOdometry;

@TeleOp(name = "Odometry Comparison", group = "Test")
public class OdoCompareTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain dt = new Drivetrain(this);
        OpticalOdometry otos = new OpticalOdometry(hardwareMap);

        Localizer localizer = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick);
        telemetry.update();

        waitForStart();

        Pose2d deadwheelPose = new Pose2d(new Vector2d(0, 0), 0);

        while (opModeIsActive()) {
            Pose2d opticalPose = otos.getCurrentPose();

            Twist2dDual<Time> twist = localizer.update();
            deadwheelPose = deadwheelPose.plus(twist.value());

            dt.update();

            telemetry.addData("optical pose", "(%.1f, %.1f) %f",
                    opticalPose.position.x, opticalPose.position.y, Math.toDegrees(opticalPose.heading.toDouble()));
            telemetry.addData("deadwheel pose", "(%.1f, %.1f) %f",
                    deadwheelPose.position.x, deadwheelPose.position.y,
                    Math.toDegrees(deadwheelPose.heading.toDouble()));

            telemetry.update();
        }
    }
}
