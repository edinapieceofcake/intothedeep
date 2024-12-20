package edu.edina.Libraries.RoadRunner.tuning.otos;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.RoadRunner.OpticalOdometry;
import edu.edina.Libraries.RoadRunner.SparkFunOTOSDrive;
import edu.edina.Libraries.Robot.Odometry;

@TeleOp
//@Disabled
public class OTOSTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Odometry odometry = new OpticalOdometry(hardwareMap);

        telemetry.addLine("OTOS Test");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            odometry.update();
            telemetry.addData("pose estimate", "%.2f, %.2f, %.2f",
                    odometry.getPoseEstimate().position.x,
                    odometry.getPoseEstimate().position.y,
                    Math.toDegrees(odometry.getPoseEstimate().heading.toDouble()));
            telemetry.update();
        }
    }
}
