package edu.edina.Libraries.Robot;

import com.qualcomm.robotcore.hardware.VoltageSensor;

import edu.edina.Libraries.RoadRunner.Localizer;

public interface DrivingRobotHardware {
    Drivetrain getDrivetrain();
    VoltageSensor getVoltageSensor();
    Odometry getOdometry();
}

