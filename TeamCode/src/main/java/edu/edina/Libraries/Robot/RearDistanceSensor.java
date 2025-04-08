package edu.edina.Libraries.Robot;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RearDistanceSensor {
    private final Rev2mDistanceSensor rightDistance, leftDistance;

    public RearDistanceSensor(HardwareMap hardwareMap) {
        rightDistance = hardwareMap.get(Rev2mDistanceSensor.class, "distance_right");
        leftDistance = hardwareMap.get(Rev2mDistanceSensor.class, "distance_left");
    }

    public double readLeftBack() {
        return leftDistance.getDistance(DistanceUnit.INCH);
    }

    public double readRightBack() {
        return rightDistance.getDistance(DistanceUnit.INCH);
    }
}