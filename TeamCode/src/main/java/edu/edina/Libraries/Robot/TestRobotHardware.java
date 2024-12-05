package edu.edina.Libraries.Robot;

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

        Localizer o;
        try {
            o = new OpticalLocalizer(hardwareMap);
        } catch (Exception x) {
            o = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick);
        }

        odometry=o;
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
