package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Rotation2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class OpticalOdometry {
    public static String name = "sensor_otos";

    public static double angularScalar = 0.99173554;
    public static double linearScalar = 1.04712042;

    //Inches
    public static double offsetX = 6.4;
    public static double offsetY = 1.6;

    //Degrees
    public static double offsetH = 180;

    private final SparkFunOTOS otos;

    public OpticalOdometry(HardwareMap hw) {
        otos = hw.get(SparkFunOTOS.class, name);

        otos.calibrateImu();
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularScalar(angularScalar);
        otos.setLinearScalar(linearScalar);
        otos.setOffset(new SparkFunOTOS.Pose2D(offsetX, offsetY, offsetH));
        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
    }

    public Pose2d getCurrentPose() {
        SparkFunOTOS.Pose2D pose = otos.getPosition();
        return new Pose2d(new Vector2d(pose.x, pose.y), Math.toRadians(pose.h));
    }

    public Pose2dDual<Time> getCurrentPoseDual() {
        SparkFunOTOS.Pose2D pose = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D vel = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D acc = new SparkFunOTOS.Pose2D();
        otos.getPosVelAcc(pose, vel, acc);
        return new Pose2dDual<Time>(
                new DualNum<Time>(new double[]{pose.x, vel.x, acc.x}),
                new DualNum<Time>(new double[]{pose.y, vel.y, acc.y}),
                new DualNum<Time>(new double[]{Math.toRadians(pose.h), Math.toRadians(vel.h), Math.toRadians(acc.h)}));
    }

    public void calibrateIMU() {
        RobotLog.ii(name, "starting OTOS calibration");
        otos.calibrateImu();
        RobotLog.ii(name, "finished OTOS calibration");
    }
}