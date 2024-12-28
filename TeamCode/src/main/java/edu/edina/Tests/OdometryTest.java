package edu.edina.Tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.RoadRunner.Localizer;
import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.RoadRunner.OpticalOdometry;
import edu.edina.Libraries.RoadRunner.ThreeDeadWheelLocalizer;
import edu.edina.Libraries.Robot.LocalizerOdometry;
import edu.edina.Libraries.Robot.Odometry;

@TeleOp
public class OdometryTest extends LinearOpMode {
    private static final boolean USE_OPTICAL = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Odometry odometry;
        if (USE_OPTICAL) {
            odometry = new OpticalOdometry(hardwareMap);
        } else {
            Localizer localizer = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick);
            odometry = new LocalizerOdometry(localizer);
        }

        waitForStart();

        while (opModeIsActive()) {
            odometry.update();
            Pose2d pose = odometry.getPoseEstimate();
            PoseVelocity2d vel = odometry.getVelocityEstimate();

            telemetry.addData("pose", "%.2f, %.2f, %.2f",
                    pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("vel", "%.2f, %.2f, %.2f",
                    vel.linearVel.x, vel.linearVel.y, Math.toDegrees(vel.angVel));
            telemetry.update();
        }
    }
}
