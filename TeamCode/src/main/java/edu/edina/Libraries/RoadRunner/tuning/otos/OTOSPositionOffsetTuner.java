package edu.edina.Libraries.RoadRunner.tuning.otos;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.RoadRunner.OpticalOdometry;
import edu.edina.Libraries.Robot.Odometry;

@TeleOp
//@Disabled
public class OTOSPositionOffsetTuner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Odometry odometry = new OpticalOdometry(hardwareMap);

        telemetry.addLine("OTOS Position Offset Tuner");
        telemetry.addLine("Line the robot against the corner of two walls facing forward and Press START.");
        telemetry.addLine("Then rotate the robot exactly 180 degrees and press it back into the corner.");
        telemetry.addLine("Finally, copy the pose offset into line 38 of SparkFunOTOSDrive.");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            odometry.update();
            telemetry.addData("Heading (deg)", Math.toDegrees(odometry.getPoseEstimate().heading.toDouble()));
            if (Math.abs(Math.toDegrees(odometry.getPoseEstimate().heading.toDouble())) > 175) {
                telemetry.addData("X Offset", odometry.getPoseEstimate().position.x / 2);
                telemetry.addData("Y Offset", odometry.getPoseEstimate().position.y / 2);
            } else {
                telemetry.addLine("Rotate the robot 180 degrees and align it to the corner again.");
            }
            telemetry.update();
        }


    }
}
