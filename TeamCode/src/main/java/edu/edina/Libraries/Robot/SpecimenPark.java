package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

import edu.edina.Libraries.PurePursuit.PurePursuit;

@Config
public class SpecimenPark {
    public SpecimenPark(RobotHardware hw) {
        double left = hw.distanceSensors.readLeftBack();
        double right = hw.distanceSensors.readRightBack();

        Vector2d[] path = new Vector2d[]{
                pursuit(left, right, false),
                pursuit(left, right, true)
        };

        // turn to field coordinates


        PurePursuit pursuit = new PurePursuit(path, false);
    }

    public static Vector2d pursuit(double left, double right, boolean near) {
        double r1 = left + SensorLayout.centerOffset;
        double r2 = right + SensorLayout.centerOffset;
        double avg = (r1 + r2) / 2;
        double delta = Math.abs(r1 - r2);
        double angle = Math.atan2(delta, SensorLayout.width);

        double distance;
        if (near) distance = 7;
        else distance = 10;

        return calcVec(avg, angle, distance);
    }

    private static Vector2d calcVec(double avg, double angle, double distance) {
        double x = avg - distance * Math.cos(angle);
        double y = -distance * Math.sin(angle);
        return new Vector2d(x, y);
    }
}