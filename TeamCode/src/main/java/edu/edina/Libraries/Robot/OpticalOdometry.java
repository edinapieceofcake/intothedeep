package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class OpticalOdometry {
    public static double angularScalar = 0.99173554;
    public static double linearScalar = 1.04712042;

    //Inches
    public static double offsetX = 6.4;
    public static double offsetY = 1.6;

    //Degrees
    public static double offsetH = 180;

    private final SparkFunOTOS otos;

    public OpticalOdometry(HardwareMap hw) {
        otos = hw.get(SparkFunOTOS.class, "sensor_otos");

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
}