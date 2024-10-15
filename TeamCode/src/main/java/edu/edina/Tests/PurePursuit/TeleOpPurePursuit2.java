package edu.edina.Tests.PurePursuit;

import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import edu.edina.Libraries.PurePursuit.PurePursuit;
import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.Robot.FieldToRobot;
import edu.edina.Libraries.Robot.RobotHardware;

@Disabled
@TeleOp
public class TeleOpPurePursuit2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware hw = new RobotHardware(hardwareMap, telemetry);

        Pose2d pose = new Pose2d(0, 0, 0);

        //only using fields, remove later
        MecanumDrive drive = new MecanumDrive(hardwareMap, pose);

        MotorFeedforward feedforward = new MotorFeedforward(drive.PARAMS.kS,
                drive.PARAMS.kV / drive.PARAMS.inPerTick, drive.PARAMS.kA / drive.PARAMS.inPerTick);

        double purePursuitX;
        double purePursuitY;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        PurePursuit pp = new PurePursuit(
                new Vector2d[]{
                        new Vector2d(0, 0),
                        new Vector2d(10, 0)
                },
                false);

        FieldToRobot robotRel = new FieldToRobot();

        waitForStart();

        while (opModeIsActive()) {
            Twist2dDual<Time> twist = hw.odometry.update();
            pose = pose.plus(twist.value());

            double yawIMU = hw.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            pp.nextPursuitPoint(pose.position, 4);

            purePursuitX = pp.getPursuitPoint().x;
            purePursuitY = pp.getPursuitPoint().y;
//            Pose2dDual<Time>

            telemetry.addData("pursuit point", "%f, %f", purePursuitX, purePursuitY);

            Vector2d rv = robotRel.toRobotRel(pose, pp.getPursuitPoint());

            telemetry.addData("robot relative vector", "%.2f, %.2f, %.1f", rv.x, rv.y, Math.toDegrees(Math.atan2(rv.y, rv.x)));

            telemetry.addData("x,    y,    h", "%.4f, %.4f, %.4f",
                    pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("imu heading", yawIMU);

            // put rv into a MotorCommand, and print the powers it would use
            PoseVelocity2d robotVelRobot = twist.velocity().value();
            PoseVelocity2dDual<Time> command;

//            MecanumKinematics.WheelVelocities<Time> wheelVels = drive.kinematics.inverse(command);

            telemetry.update();
        }
    }
}