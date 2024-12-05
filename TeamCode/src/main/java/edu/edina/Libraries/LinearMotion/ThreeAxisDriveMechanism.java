package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import edu.edina.Libraries.PurePursuit.PurePursuit;
import edu.edina.Libraries.RoadRunner.Localizer;
import edu.edina.Libraries.Robot.Drivetrain;
import edu.edina.Libraries.Robot.DrivingRobotHardware;
import edu.edina.Libraries.Robot.FieldToRobot;
import edu.edina.Libraries.Robot.Odometry;
import edu.edina.Libraries.Robot.RobotHardware;
import kotlin.NotImplementedError;

@Config
public class ThreeAxisDriveMechanism {
    private static final boolean LOG = true;
    private static final String TAG = "3-axis-drive";
    public static double LATERAL_MULT = 0.82;

    private Odometry odometry;
    private Drivetrain drivetrain;
    private VoltageSensor vs;
    private Vector2d robotRelVel;
    private PurePursuit purePursuit;
    private AxialMechanism axial;
    private LateralMechanism lateral;
    private LinearMotionController axialCon, lateralCon, rotCon;
    private double pursuitRadius;

    public ThreeAxisDriveMechanism(DrivingRobotHardware hw) {
        drivetrain = hw.getDrivetrain();
        odometry = hw.getOdometry();
        vs = hw.getVoltageSensor();

        axial = new AxialMechanism();
        lateral = new LateralMechanism();
        axialCon = new LinearMotionController(axial);
        lateralCon = new LinearMotionController(lateral);

        pursuitRadius = 6;
    }

    public void setPursuitRadius(double radius) {
        pursuitRadius = radius;
    }

    public double getPursuitRadius() {
        return pursuitRadius;
    }

    public void setPath(Vector2d[] path, boolean closed) {
        purePursuit = new PurePursuit(path, closed);

        if (LOG)
            RobotLog.ii(TAG, "setPath to %s", purePursuit);
    }

    public void update() {
        Pose2d pose = odometry.getPoseEstimate();
        PoseVelocity2d vel = odometry.getVelocityEstimate();

        if (purePursuit == null)
            return;

        // set pursuit

        purePursuit.nextPursuitPoint(pose.position, pursuitRadius);

        Vector2d pursuit = purePursuit.getPursuitPoint();
        Vector2d robotRelPursuitPoint = FieldToRobot.toRobotRel(pose, pursuit);

        axialCon.setTarget(robotRelPursuitPoint.x);
        lateralCon.setTarget(robotRelPursuitPoint.y);
        // yawCon.setTarget();

        // calculate velocities
        Vector2d fieldVel = vel.linearVel;
        robotRelVel = FieldToRobot.toRobotRel(pose, fieldVel);

        // use the LinearMotionController to calculate powers
        axialCon.run();
        lateralCon.run();
        // yawCon.run();

        drivetrain.update(axial.power, -lateral.power * LATERAL_MULT, 0);

        if (LOG) {
            RobotLog.ii(TAG, "pursuit point=(%.1f, %.1f), robot rel=(%.1f, %.1f)",
                    pursuit.x, pursuit.y,
                    robotRelPursuitPoint.x, robotRelPursuitPoint.y);

            RobotLog.ii(TAG, "linear velocity=(%.1f, %.1f), robot rel=(%.1f, %.1f)",
                    fieldVel.x, fieldVel.y,
                    robotRelVel.x, robotRelVel.y);

            RobotLog.ii(TAG, "drive power: axial=%.3f, lateral=%.3f, yaw=%.3f",
                    axial.power, lateral.power, 0.0);
        }
    }

    public class AxialMechanism implements ILinearMechanism {
        public double power;

        @Override
        public void setPower(double power) {
            this.power = power;
        }

        @Override
        public double getPosition(boolean raw) {
            throw new NotImplementedError();
        }

        @Override
        public DualNum<Time> getPositionAndVelocity(boolean raw) {
            return new DualNum<Time>(new double[]{0, robotRelVel.x});
        }

        @Override
        public LinearMechanismSettings getSettings() {
            return AxialDriveMechanism.getStaticSettings();
        }
    }

    public class LateralMechanism implements ILinearMechanism {
        public double power;

        @Override
        public void setPower(double power) {
            this.power = power;
        }

        @Override
        public double getPosition(boolean raw) {
            throw new NotImplementedError();
        }

        @Override
        public DualNum<Time> getPositionAndVelocity(boolean raw) {
            return new DualNum<Time>(new double[]{0, robotRelVel.y});
        }

        @Override
        public LinearMechanismSettings getSettings() {
            return LateralDriveMechanism.getStaticSettings();
        }
    }

    public class RotationalMechanism implements ILinearMechanism {
        public double power;

        public RotationalMechanism() {

        }

        @Override
        public void setPower(double power) {
            this.power = power;
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
            return RotationalDriveMechanism.getStaticSettings();
        }
    }
}