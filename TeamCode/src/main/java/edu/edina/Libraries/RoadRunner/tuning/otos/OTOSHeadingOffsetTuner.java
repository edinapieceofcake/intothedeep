package edu.edina.Libraries.RoadRunner.tuning.otos;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.RoadRunner.OpticalOdometry;
import edu.edina.Libraries.Robot.Odometry;

@TeleOp
@Disabled
//@Disabled
public class OTOSHeadingOffsetTuner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Odometry odometry = new OpticalOdometry(hardwareMap);

        telemetry.addLine("OTOS Heading Offset Tuner");
        telemetry.addLine("Line the side of the robot against a wall and Press START.");
        telemetry.addLine("Then push the robot forward some distance.");
        telemetry.addLine("Finally, copy the heading offset into line 38 of SparkFunOTOSDrive");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            odometry.update();
            Pose2d pose = odometry.getPoseEstimate();
            telemetry.addData("Heading Offset (radians, enter this one into SparkFunOTOSDrive!)", Math.atan2(pose.position.y, pose.position.x));
            telemetry.addData("Heading Offset (degrees)", Math.toDegrees(Math.atan2(pose.position.y, pose.position.x)));
            telemetry.update();
        }


    }
}
