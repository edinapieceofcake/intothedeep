package edu.edina.Libraries.LinearMotion;

import static edu.edina.Libraries.LinearMotion.LinearMechanismSettings.mv;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import edu.edina.Libraries.RoadRunner.ThreeDeadWheelLocalizer;
import edu.edina.Libraries.Robot.Drivetrain;
import edu.edina.Libraries.Robot.RobotHardware;

public class AxialDriveMechanism implements ILinearMechanism {
    private ThreeDeadWheelLocalizer odometry;
    private Drivetrain drivetrain;
    private VoltageSensor vs;
    private Pose2d pose;

    public static final LinearMechanismSettings SETTINGS = new LinearMechanismSettings(
            "axial drive", "inches",
            6.6229e-2, 1.5638e-2, 3.4678e-3, 20,
            1 / (2 * 3.4678e-3),
            0.2,
            0.25,
            0.3,
            4.26e+4);

    public AxialDriveMechanism(RobotHardware hw) {
        drivetrain = hw.drivetrain;
        odometry = hw.odometry;
        pose = new Pose2d(new Vector2d(0, 0), 0);

        vs = hw.voltageSensor;
    }

    @Override
    public void setPower(double power) {
        double actualPower = power * 12 / vs.getVoltage();
        drivetrain.update(actualPower, 0, 0);
    }

    @Override
    public double getPosition(boolean raw) {
        Twist2dDual<Time> t = odometry.update();
        pose = pose.plus(t.value());
        return pose.position.x;
    }

    @Override
    public DualNum<Time> getPositionAndVelocity(boolean raw) {
        Twist2dDual<Time> t = odometry.update();
        pose = pose.plus(t.value());
        double v = t.velocity().linearVel.x.value();
        return new DualNum<Time>(new double[]{pose.position.x, v});
    }

    @Override
    public LinearMechanismSettings getSettings() {
        return SETTINGS;
    }
}
