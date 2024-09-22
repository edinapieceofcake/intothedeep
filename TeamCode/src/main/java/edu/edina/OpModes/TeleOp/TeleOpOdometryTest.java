package edu.edina.OpModes.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.RoadRunner.ThreeDeadWheelLocalizer;
import edu.edina.Libraries.Robot.RobotHardware;

@TeleOp
public class TeleOpOdometryTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        RobotHardware hw = new RobotHardware(hardwareMap, telemetry);

        Pose2d pose = new Pose2d(0, 0, 0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {
            double max;

            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            hw.leftFrontDrive.setPower(leftFrontPower);
            hw.rightFrontDrive.setPower(rightFrontPower);
            hw.leftBackDrive.setPower(leftBackPower);
            hw.rightBackDrive.setPower(rightBackPower);

            Twist2dDual<Time> t = hw.odometry.update();
            pose = pose.plus(t.value());

            double yawIMU = hw.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("x,    y,    h", "%.4f, %.4f, %.4f",
                    pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("imu heading", yawIMU);
            telemetry.update();
        }
    }
}