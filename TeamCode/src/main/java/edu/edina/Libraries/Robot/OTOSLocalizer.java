package edu.edina.Libraries.Robot;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.robotcore.hardware.HardwareMap;

import edu.edina.Libraries.RoadRunner.Localizer;


public class OTOSLocalizer implements Localizer {
    ;
    OpticalOdometry opticalOdometry;
    Pose2dDual<Time> lastOdometryReading;

    public OTOSLocalizer(HardwareMap hardwareMap) {
        opticalOdometry = new OpticalOdometry(hardwareMap);
    }

    @Override
    public Twist2dDual<Time> update() {
        Pose2dDual<Time> currentOdometryReading = opticalOdometry.getCurrentPoseDual();



        double dx, vx;
        double dy, vy;
        double da, va;
        if (lastOdometryReading == null){
            dx = 0;
            vx = 0;
            dy = 0;
            vy = 0;
            da = 0;
            va = 0;
        }else {
            dx = currentOdometryReading.position.x.get(0) - lastOdometryReading.position.x.get(0);
            vx = currentOdometryReading.position.x.get(1);
            dy = currentOdometryReading.position.y.get(0) - lastOdometryReading.position.y.get(0);
            vy = currentOdometryReading.position.y.get(1);
            da = currentOdometryReading.heading.value().toDouble();
            va = currentOdometryReading.heading.velocity().get(1);
        }

        DualNum<Time> xdual = new DualNum<>(new double[]{dx, vx});
        DualNum<Time> ydual = new DualNum<>(new double[]{dy, vy});
        DualNum<Time> adual = new DualNum<>(new double[]{da, va});

        lastOdometryReading = currentOdometryReading;
        return new Twist2dDual<>(new Vector2dDual<>(xdual, ydual), adual);
    }
}
