package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;

import edu.edina.Libraries.RoadRunner.ThreeDeadWheelLocalizer;
import edu.edina.Libraries.Robot.Drivetrain;
import edu.edina.Libraries.Robot.RobotHardware;
import kotlin.NotImplementedError;

public class DriveMechanisms {
    private ThreeDeadWheelLocalizer odometry;
    private Drivetrain drivetrain;

    private static final LinearMechanismSettings AXIAL_SETTINGS = new LinearMechanismSettings(
            0,
            0,
            0,
            0);
    private static final LinearMechanismSettings LATERAL_SETTINGS = new LinearMechanismSettings(
            0,
            0,
            0,
            0);
    private static final LinearMechanismSettings ROTATIONAL_SETTINGS = new LinearMechanismSettings(
            0,
            0,
            0,
            0);

    public DriveMechanisms(RobotHardware hw) {
        drivetrain = hw.drivetrain;
        odometry = hw.odometry;
    }

    public void update(){

    }

    public class AxialMechanism implements ILinearMechanism {
        public AxialMechanism() {

        }

        @Override
        public void setPower(double power) {

        }

        @Override
        public double getPosition(boolean raw) {
            return 0;
        }


        @Override
        public DualNum<Time> getPositionAndVelocity(boolean raw) {
            throw new NotImplementedError();
        }

        @Override
        public LinearMechanismSettings getSettings() {
            return null;
        }
    }

    public class LateralMechanism implements ILinearMechanism {
        public LateralMechanism() {

        }

        @Override
        public void setPower(double power) {

        }

        @Override
        public double getPosition(boolean raw) {
            return 0;
        }

        @Override
        public DualNum<Time> getPositionAndVelocity(boolean raw) {
            throw new NotImplementedError();
        }

        @Override
        public LinearMechanismSettings getSettings() {
            return null;
        }
    }

    public class RotationalMechanism implements ILinearMechanism {
        public RotationalMechanism() {

        }

        @Override
        public void setPower(double power) {

        }

        @Override
        public double getPosition(boolean raw) {
            return 0;
        }

        @Override
        public DualNum<Time> getPositionAndVelocity(boolean raw) {
            throw new NotImplementedError();
        }

        @Override
        public LinearMechanismSettings getSettings() {
            return null;
        }
    }
}