package edu.edina.Tests.PurePursuit;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import edu.edina.Libraries.PurePursuit.PurePursuit;
import edu.edina.Libraries.Robot.FieldToRobot;
import edu.edina.Libraries.Robot.RobotHardware;

@TeleOp
public class TeleOpPurePursuit extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        RobotHardware hw = new RobotHardware(hardwareMap, telemetry);

        Pose2d pose = new Pose2d(0, 0, 0);

        double purePursuitX;
        double purePursuitY;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        PurePursuit pp = new PurePursuit(new Vector2d[]{
                new Vector2d(0, 0), new Vector2d(30, 0), new Vector2d(30, 20)
        });

        FieldToRobot robotRel = new FieldToRobot();

        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {
            double max;

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            hw.leftFrontDrive.setPower(leftFrontPower * 0.3);
            hw.rightFrontDrive.setPower(rightFrontPower * 0.3);
            hw.leftBackDrive.setPower(leftBackPower * 0.3);
            hw.rightBackDrive.setPower(rightBackPower * 0.3);

            Twist2dDual<Time> t = hw.odometry.update();
            pose = pose.plus(t.value());

            double yawIMU = hw.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            pp.nextPursuitPoint(pose.position, 10);

            purePursuitX = pp.getPursuitPoint().x;
            purePursuitY = pp.getPursuitPoint().y;

            telemetry.addData("pursuit point", "%f, %f", purePursuitX, purePursuitY);

            Vector2d rv = robotRel.toRobotRel(pose, pp.getPursuitPoint());

            telemetry.addData("robot relative vector", "%.2f, %.2f", rv.x, rv.y);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("x,    y,    h", "%.4f, %.4f, %.4f",
                    pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("imu heading", yawIMU);
            telemetry.update();
        }
    }
}
