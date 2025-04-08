package edu.edina.Libraries.RoadRunner.tuning.otos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.RoadRunner.OpticalOdometry;
import edu.edina.Libraries.Robot.Odometry;

@TeleOp
@Disabled
//@Disabled
public class OTOSAngularScalar extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Odometry odometry = new OpticalOdometry(hardwareMap);

        double radsTurned = 0;
        Rotation2d lastHeading = Rotation2d.fromDouble(0);
        telemetry.addLine("OTOS Angular Scalar Tuner");
        telemetry.addLine("Press START, then rotate the robot on the ground 10 times (3600 degrees).");
        telemetry.addLine("Then copy the scalar into OpticalOdometry.");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            odometry.update();
            radsTurned += odometry.getPoseEstimate().heading.minus(lastHeading);
            lastHeading = odometry.getPoseEstimate().heading;
            telemetry.addData("Uncorrected Degrees Turned", Math.toDegrees(radsTurned));
            telemetry.addData("Calculated Angular Scalar", 3600 / Math.toDegrees(radsTurned));
            telemetry.update();
        }
    }
}
