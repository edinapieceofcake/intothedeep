package edu.edina.Tests.PurePursuit;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import edu.edina.Libraries.PurePursuit.PurePursuit;
import edu.edina.Libraries.Robot.FieldToRobot;
import edu.edina.Libraries.Robot.RobotHardware;

@TeleOp
@Disabled
public class TeleOpPurePursuit extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);

        Vector2d rv = new Vector2d(0, 0);

        Pose2d pose = new Pose2d(0, 0, 0);

        double purePursuitX = 0;
        double purePursuitY = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        PurePursuit pp = new PurePursuit(
                new Vector2d[]{
                        new Vector2d(0, 0),
                        new Vector2d(10, 0),
                        new Vector2d(10, -10),
                        new Vector2d(0, -10),
                        new Vector2d(0, 0)
                },
                true);

        PurePursuit pp2 = new PurePursuit(
                new Vector2d[]{
                        new Vector2d(0, -5),
                        new Vector2d(5, 0),
                        new Vector2d(10, -5),
                        new Vector2d(5, -10)
                },
                true);

        FieldToRobot robotRel = new FieldToRobot();

        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {
            Twist2dDual<Time> t = hw.odometry.update();
            pose = pose.plus(t.value());

            double yawIMU = hw.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            if (gamepad1.a) {
                pp.nextPursuitPoint(pose.position, 3);
                purePursuitX = pp.getPursuitPoint().x;
                purePursuitY = pp.getPursuitPoint().y;
                rv = robotRel.toRobotRel(pose, pp.getPursuitPoint());
            } else if (gamepad1.x) {
                pp2.nextPursuitPoint(pose.position, 3);
                purePursuitX = pp2.getPursuitPoint().x;
                purePursuitY = pp2.getPursuitPoint().y;
                rv = robotRel.toRobotRel(pose, pp2.getPursuitPoint());
            }

            telemetry.addData("pursuit point", "%f, %f", purePursuitX, purePursuitY);

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
            
            telemetry.addData("\n\nmotor cmd", mc.toString());
            /*
            hw.leftFrontDrive.setPower(mc.getLeftFrontPower());
            hw.rightFrontDrive.setPower(mc.getRightFrontPower());
            hw.leftBackDrive.setPower(mc.getLeftBackPower());
            hw.rightBackDrive.setPower(mc.getRightBackPower());
            */
            telemetry.update();
        }
    }
}