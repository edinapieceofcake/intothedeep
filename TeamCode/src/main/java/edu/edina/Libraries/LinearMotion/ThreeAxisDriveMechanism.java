package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import edu.edina.Libraries.PurePursuit.PurePursuit;
import edu.edina.Libraries.Robot.Drivetrain;
import edu.edina.Libraries.Robot.DrivingRobotHardware;
import edu.edina.Libraries.Robot.FieldToRobot;
import edu.edina.Libraries.Robot.Odometry;
import kotlin.NotImplementedError;

@Config
public class ThreeAxisDriveMechanism {
    private static final boolean LOG = false;
    private static final String TAG = "3-axis-drive";
    public static double LATERAL_MULT = 0.82;

    private final Destination destination;
    private final Odometry odometry;
    private final Drivetrain drivetrain;
    private final VoltageSensor vs;
    private Pose2d robotRelVel;
    private final LinearMotionController axialCon, lateralCon, yawCon;
    private double axialPower, lateralPower, rotationalPower;

    public ThreeAxisDriveMechanism(DrivingRobotHardware hw, Destination destination) {
        drivetrain = hw.getDrivetrain();
        odometry = hw.getOdometry();
        vs = hw.getVoltageSensor();

        AxialMechanism axial = new AxialMechanism();
        LateralMechanism lateral = new LateralMechanism();
        RotationalMechanism rotation = new RotationalMechanism();

        axialCon = new LinearMotionController(axial);
        lateralCon = new LinearMotionController(lateral);
        yawCon = new LinearMotionController(rotation);

        this.destination = destination;
    }

    public void update() {
        odometry.update();
        Pose2d pose = odometry.getPoseEstimate();
        PoseVelocity2d vel = odometry.getVelocityEstimate();

        // set pursuit

        Vector2d pursuit = destination.getDestination(pose);
        Vector2d robotRelPursuitPoint = FieldToRobot.toRobotRel(pose, pursuit);
        Rotation2d pursuitHeading = destination.heading(pose);

        axialCon.setTarget(robotRelPursuitPoint.x);
        lateralCon.setTarget(robotRelPursuitPoint.y);
        yawCon.setTarget(Math.toDegrees(pursuitHeading.toDouble()));

        // calculate velocities
        Vector2d fieldVel = vel.linearVel;
        robotRelVel = new Pose2d(FieldToRobot.toRobotRel(pose, fieldVel), vel.angVel);

        // use the LinearMotionController to calculate powers
        axialCon.run();
        lateralCon.run();
        yawCon.run();

        drivetrain.update(axialPower, lateralPower, rotationalPower);

        if (LOG) {
            RobotLog.ii(TAG, "pursuit point=(%.1f, %.1f), robot rel=(%.1f, %.1f)",
                    pursuit.x, pursuit.y,
                    robotRelPursuitPoint.x, robotRelPursuitPoint.y);

            RobotLog.ii(TAG, "linear velocity=(%.1f, %.1f), robot rel=(%.1f, %.1f)",
                    fieldVel.x, fieldVel.y,
                    robotRelVel.position.x, robotRelVel.position.y);

            RobotLog.ii(TAG, "drive power: axial=%.3f, lateral=%.3f, yaw=%.3f",
                    axialPower, lateralPower, rotationalPower);
        }
    }

    public class AxialMechanism implements ILinearMechanism {
        @Override
        public void setPower(double power) {
            axialPower = power;
        }

        @Override
        public double getPosition(boolean raw) {
            throw new NotImplementedError();
        }

        @Override
        public DualNum<Time> getPositionAndVelocity(boolean raw) {
            return new DualNum<Time>(new double[]{0, robotRelVel.position.x});
        }

        @Override
        public LinearMechanismSettings getSettings() {
            return AxialDriveMechanism.getStaticSettings();
        }
    }

    public class LateralMechanism implements ILinearMechanism {
        @Override
        public void setPower(double power) {
            lateralPower = -power * LATERAL_MULT;
        }

        @Override
        public double getPosition(boolean raw) {
            throw new NotImplementedError();
        }

        @Override
        public DualNum<Time> getPositionAndVelocity(boolean raw) {
            return new DualNum<Time>(new double[]{0, robotRelVel.position.y});
        }

        @Override
        public LinearMechanismSettings getSettings() {
            return LateralDriveMechanism.getStaticSettings();
        }
    }

    public class RotationalMechanism implements ILinearMechanism {
        @Override
        public void setPower(double power) {
            rotationalPower = power;
        }

        @Override
        public double getPosition(boolean raw) {
            return 0;
        }

        @Override
        public DualNum<Time> getPositionAndVelocity(boolean raw) {
            return new DualNum<Time>(new double[]{0, Math.toDegrees(robotRelVel.heading.toDouble())});
        }

        @Override
        public LinearMechanismSettings getSettings() {
            return RotationalDriveMechanism.getStaticSettings();
        }
    }
}