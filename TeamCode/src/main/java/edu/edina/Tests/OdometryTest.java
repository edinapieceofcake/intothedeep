package edu.edina.Tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.RoadRunner.OpticalOdometry;
import edu.edina.Libraries.Robot.Odometry;

@TeleOp
public class OdometryTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Odometry odometry = new OpticalOdometry(hardwareMap);

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
