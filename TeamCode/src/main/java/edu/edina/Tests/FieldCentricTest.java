package edu.edina.Tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.Robot.Drivetrain;
import edu.edina.Libraries.Robot.FieldToRobot;
import edu.edina.Libraries.Robot.OpticalOdometry;

@TeleOp
public class FieldCentricTest extends LinearOpMode {
    private Drivetrain dt;

    @Override
    public void runOpMode() throws InterruptedException {
        dt = new Drivetrain(this);
        OpticalOdometry odo = new OpticalOdometry(hardwareMap);

        waitForStart();

        Pose2d currPose = odo.getCurrentPose();
        Pose2d refPose = currPose;

        while (opModeIsActive()) {
            Pose2dDual<Time> poseDual = odo.getCurrentPoseDual();
            currPose = poseDual.value();

            Vector2d vel = poseDual.position.drop(1).value();

            Vector2d c = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
            Vector2d p1 = currPose.position;
            Vector2d q0 = refPose.position;
            Vector2d q1 = project(p1.minus(q0), c).plus(q0);

            refPose = new Pose2d(q1, refPose.heading);

            Vector2d pursuitPoint = q1.plus(c);

            Vector2d relPursuitPoint = FieldToRobot.toRobotRel(currPose, pursuitPoint);

            dt.update(.4 * relPursuitPoint.x, .4 * -relPursuitPoint.y, gamepad1.right_stick_x);

            telemetry.addData("c", "(%.1f, %.1f)", c.x, c.y);
            telemetry.addData("pose", "(%.1f, %.1f) %f", p1.x, p1.y,
                    Math.toDegrees(currPose.heading.toDouble()));
            telemetry.addData("refPose", "(%.1f, %.1f) %f", q1.x, q1.y,
                    Math.toDegrees(refPose.heading.toDouble()));
            telemetry.addData("pursuit", "(%.1f, %.1f)", pursuitPoint.x, pursuitPoint.y);
            telemetry.addData("rel pursuit", "(%.1f, %.1f)", relPursuitPoint.x, relPursuitPoint.y);
            telemetry.update();
        }
    }

    public Vector2d project(Vector2d v, Vector2d onto) {
        double o2 = onto.dot(onto);
        if (o2 == 0)
            return v;
        else
            return onto.times(v.dot(onto) / o2);
    }
}