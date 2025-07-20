package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class OpticalOdometry {
    public static String name = "sensor_otos";

    public static double angularScalar = 0.99173554;
    public static double LINEAR_MULTIPLIER = 1.12;

    //Inches
    public static double offsetX = 6.4;
    public static double offsetY = -1.6;

    //Degrees
    public static double HEADING_OFFSET_DEG = 90;

    private final SparkFunOTOS otos;

    public OpticalOdometry(HardwareMap hw, Pose2d initPose) {
        otos = hw.get(SparkFunOTOS.class, name);

        otos.calibrateImu();
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularScalar(angularScalar);
        otos.setLinearScalar(1);
        SparkFunOTOS.Pose2D o = otos.getOffset(); // zero it out!
        otos.setOffset(new SparkFunOTOS.Pose2D(-o.x, -o.y, -o.h));
        otos.setOffset(ConvertPose(new Pose2d(offsetX, offsetY, Math.toRadians(HEADING_OFFSET_DEG))));
        otos.setPosition(ConvertPose(initPose));
    }

    public OpticalOdometry(HardwareMap hw) {
        this(hw, new Pose2d(0, 0, 0));
    }

    public Pose2dDual<Time> getCurrentPoseDual() {
        SparkFunOTOS.Pose2D pose = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D vel = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D acc = new SparkFunOTOS.Pose2D();
        otos.getPosVelAcc(pose, vel, acc);
        return new Pose2dDual<Time>(
                new DualNum<Time>(new double[]{LINEAR_MULTIPLIER * pose.x, LINEAR_MULTIPLIER * vel.x, acc.x}),
                new DualNum<Time>(new double[]{LINEAR_MULTIPLIER * pose.y, LINEAR_MULTIPLIER * vel.y, acc.y}),
                new DualNum<Time>(new double[]{Math.toRadians(pose.h), Math.toRadians(vel.h), Math.toRadians(acc.h)}));
    }

    public void calibrateIMU() {
        RobotLog.ii(name, "starting OTOS calibration");
        otos.calibrateImu();
        RobotLog.ii(name, "finished OTOS calibration");
    }

    private static SparkFunOTOS.Pose2D ConvertPose(Pose2d pose) {
        return new SparkFunOTOS.Pose2D(
                pose.position.x / LINEAR_MULTIPLIER,
                pose.position.y / LINEAR_MULTIPLIER,
                Math.toDegrees(pose.heading.toDouble()));
    }
}