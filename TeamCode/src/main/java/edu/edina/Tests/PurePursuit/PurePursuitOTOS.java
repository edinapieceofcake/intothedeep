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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import edu.edina.Libraries.PurePursuit.PurePursuit;
import edu.edina.Libraries.Robot.FieldToRobot;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.SparkFunOTOS;
import edu.edina.Libraries.Robot.SparkFunOTOSCorrected;

@Disabled
@TeleOp
public class PurePursuitOTOS extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    SparkFunOTOS myOtos;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);
        myOtos = hardwareMap.get(SparkFunOTOSCorrected.class, "sensor_otos");

        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        myOtos.setLinearScalar(0.87);
        myOtos.setAngularScalar(1.0);

        myOtos.calibrateImu();

        myOtos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();

        Pose2d pose = new Pose2d(0, 0, 0);

        double purePursuitX;
        double purePursuitY;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        PurePursuit pp = new PurePursuit(
                new Vector2d[]{
                        new Vector2d(0, 0),
                        new Vector2d(10, 0),
                        new Vector2d(10, -10)
                },
                false);

        FieldToRobot robotRel = new FieldToRobot();

        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {
            pose = new Pose2d(-myOtos.getPosition().x, -myOtos.getPosition().y, myOtos.getPosition().h);

            double yawIMU = hw.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            pp.nextPursuitPoint(pose.position, 5);

            purePursuitX = pp.getPursuitPoint().x;
            purePursuitY = pp.getPursuitPoint().y;

            telemetry.addData("pursuit point", "%f, %f", purePursuitX, purePursuitY);

            Vector2d rv = robotRel.toRobotRel(pose, pp.getPursuitPoint());

            telemetry.addData("robot relative vector", "%.2f, %.2f, %.1f", rv.x, rv.y, Math.toDegrees(Math.atan2(rv.y, rv.x)));

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("x,    y,    h", "%.4f, %.4f, %.4f",
                    pose.position.x, pose.position.y, pose.heading.toDouble());
            telemetry.addData("imu heading", yawIMU);

            // put rv into a MotorCommand, and print the powers it would use
            double axial = rv.x;
            double lateral = rv.y;
            double yaw = pose.heading.toDouble();

            MotorCommand mc = new MotorCommand(axial, lateral, yaw);

            if (!gamepad1.b) {
                mc.scale(0.3);
            }

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