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
                new Vector2d(0, 0), new Vector2d(10, 0), new Vector2d(10, -10)
        });

        FieldToRobot robotRel = new FieldToRobot();

        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {
            Twist2dDual<Time> t = hw.odometry.update();
            pose = pose.plus(t.value());

            double yawIMU = hw.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            pp.nextPursuitPoint(pose.position, 4);

            purePursuitX = pp.getPursuitPoint().x;
            purePursuitY = pp.getPursuitPoint().y;

            telemetry.addData("pursuit point", "%f, %f", purePursuitX, purePursuitY);

            Vector2d rv = robotRel.toRobotRel(pose, pp.getPursuitPoint());

            telemetry.addData("robot relative vector", "%.2f, %.2f, %.1f", rv.x, rv.y, Math.toDegrees(Math.atan2(rv.y, rv.x)));

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("x,    y,    h", "%.4f, %.4f, %.4f",
                    pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("imu heading", yawIMU);

            // put rv into a MotorCommand, and print the powers it would use
            double axial = rv.x;
            double lateral = rv.y;
            double yaw = Math.toDegrees(pose.heading.toDouble());

            MotorCommand mc = new MotorCommand(axial, lateral, yaw);

            if (!gamepad1.b) {
                mc.scale(0.3);
            }

            telemetry.addData("\n\nmotor cmd", mc.toString());

            hw.leftFrontDrive.setPower(mc.getLeftFrontPower());
            hw.rightFrontDrive.setPower(mc.getRightFrontPower());
            hw.leftBackDrive.setPower(mc.getLeftBackPower());
            hw.rightBackDrive.setPower(mc.getRightBackPower());

            telemetry.update();
        }
    }
}