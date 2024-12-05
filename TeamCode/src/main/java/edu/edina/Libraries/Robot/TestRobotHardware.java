package edu.edina.Libraries.Robot;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import edu.edina.Libraries.RoadRunner.Localizer;
import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.RoadRunner.OpticalLocalizer;
import edu.edina.Libraries.RoadRunner.ThreeDeadWheelLocalizer;

public class TestRobotHardware implements DrivingRobotHardware {
    public final Localizer odometry;
    public final VoltageSensor voltageSensor;
    public final Drivetrain drivetrain;

    public TestRobotHardware(LinearOpMode opMode) throws InterruptedException {
        HardwareMap hardwareMap = opMode.hardwareMap;

        drivetrain = new Drivetrain(opMode);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        if (OpticalLocalizer.isMapped(hardwareMap))
            odometry = new OpticalLocalizer(hardwareMap);
        else
            odometry = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick);
    }

    @Override
    public Localizer getOdometry() {
        return odometry;
    }

    @Override
    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    @Override
    public VoltageSensor getVoltageSensor() {
        return voltageSensor;
    }
}
