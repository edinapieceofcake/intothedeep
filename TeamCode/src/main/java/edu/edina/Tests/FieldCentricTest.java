package edu.edina.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.Angle;
import edu.edina.Libraries.Robot.Drivetrain;
import edu.edina.Libraries.Robot.FieldToRobot;
import edu.edina.Libraries.Robot.OpticalOdometry;

@Config
@TeleOp
public class FieldCentricTest extends LinearOpMode {
    private Drivetrain dt;
    public static double VEC_TRACK_ANGLE = 30;
    public static double MAX_PURSUIT_INCHES = 10;
    public static double YAW_DEADZONE = 0.1;
    public static double HEADING_P_COEF = 0.5;

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

            //get control stick vector
            Vector2d c = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x);

            //only do correction if not turning sharply
            Vector2d newRefPos;
            if (angleBetweenDeg(c, vel) < VEC_TRACK_ANGLE) {
                telemetry.addLine("vector tracking");
                Vector2d p1 = currPose.position;
                Vector2d q0 = refPose.position;
                newRefPos = project(p1.minus(q0), c).plus(q0);
            } else {
                telemetry.addLine("NOT vector tracking");
                newRefPos = currPose.position;
            }

            //angular correction unless being manually controlled
            double yawPower;
            Rotation2d newRefHead;
            if (Math.abs(gamepad1.right_stick_x) > YAW_DEADZONE) {
                yawPower = gamepad1.right_stick_x;
                newRefHead = currPose.heading;
            } else {
                double radDiff = Angle.radianDiff(currPose.heading.toDouble(), refPose.heading.toDouble());
                yawPower = HEADING_P_COEF * radDiff;
                newRefHead = refPose.heading;
            }

            //update reference pose with power
            refPose = new Pose2d(newRefPos, newRefHead);

            //smooth the correction
            Vector2d pursuitPoint = refPose.position.plus(c.times(MAX_PURSUIT_INCHES));

            //turn to robot centric and get driving power
            Vector2d relPursuitPoint = FieldToRobot.toRobotRel(currPose, pursuitPoint);
            Vector2d drivePower = relPursuitPoint.times(c.norm());

            dt.update(drivePower.x, -drivePower.y, yawPower);

            telemetry.addData("c", "(%.1f, %.1f)", c.x, c.y);
            telemetry.addData("pose", "(%.1f, %.1f) %f", currPose.position.x, currPose.position.y,
                    Math.toDegrees(currPose.heading.toDouble()));
            telemetry.addData("refPose", "(%.1f, %.1f) %f", refPose.position.x, refPose.position.y,
                    Math.toDegrees(refPose.heading.toDouble()));
            telemetry.addData("pursuit", "(%.1f, %.1f)", pursuitPoint.x, pursuitPoint.y);
            telemetry.addData("rel pursuit", "(%.1f, %.1f)", relPursuitPoint.x, relPursuitPoint.y);
            telemetry.update();
        }
    }

    private Vector2d project(Vector2d v, Vector2d onto) {
        double o2 = onto.dot(onto);
        if (o2 == 0)
            return v;
        else
            return onto.times(v.dot(onto) / o2);
    }

    private Vector2d normalize(Vector2d v) {
        double norm = v.norm();
        if (norm == 0) return v;
        else return v.div(norm);
    }

    private double angleBetweenDeg(Vector2d v0, Vector2d v1) {
        double rad = Math.acos(normalize(v0).dot(normalize(v1)));
        return Math.toDegrees(rad);
    }
}