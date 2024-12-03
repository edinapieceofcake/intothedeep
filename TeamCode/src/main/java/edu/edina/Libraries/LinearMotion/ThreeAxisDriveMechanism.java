package edu.edina.Libraries.LinearMotion;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import edu.edina.Libraries.PurePursuit.PurePursuit;
import edu.edina.Libraries.RoadRunner.ThreeDeadWheelLocalizer;
import edu.edina.Libraries.Robot.Drivetrain;
import edu.edina.Libraries.Robot.FieldToRobot;
import edu.edina.Libraries.Robot.RobotHardware;
import kotlin.NotImplementedError;

public class ThreeAxisDriveMechanism {
    private static final boolean LOG = true;
    private static final String TAG = "3-axis-drive";

    private ThreeDeadWheelLocalizer odometry;
    private Drivetrain drivetrain;
    private VoltageSensor vs;
    private Pose2d pose;
    private Vector2d robotRelVel;
    private PurePursuit purePursuit;
    private AxialMechanism axial;
    private LinearMotionController axialCon;

    public ThreeAxisDriveMechanism(RobotHardware hw) {
        drivetrain = hw.drivetrain;
        odometry = hw.odometry;
        vs = hw.voltageSensor;

        pose = new Pose2d(new Vector2d(0, 0), 0);

        axial = new AxialMechanism();
        axialCon = new LinearMotionController(axial);
    }

    public void setPath(Vector2d[] path, boolean closed) {
        purePursuit = new PurePursuit(path, closed);

        if (LOG)
            RobotLog.ii(TAG, "setPath to %s", purePursuit);
    }

    public void update() {
        Twist2dDual<Time> twist = odometry.update();
        pose = pose.plus(twist.value());

        if (purePursuit == null)
            return;

        // set pursuit
        Vector2d pursuit = purePursuit.getPursuitPoint();
        Vector2d robotRelPursuitPoint = FieldToRobot.toRobotRel(pose, pursuit);
        axialCon.setTarget(robotRelPursuitPoint.x);
        // lateralCon.setTarget();
        // yawCon.setTarget();

        // calculate velocities
        Vector2d fieldVel = new Vector2d(
                twist.velocity().linearVel.x.value(),
                twist.velocity().linearVel.y.value());
        robotRelVel = FieldToRobot.toRobotRel(pose, fieldVel);

        // use the LinearMotionController to calculate powers
        axialCon.run();
        // lateralCon.run();
        // yawCon.run();

        drivetrain.update(axial.power, 0, 0);

        if (LOG) {
            RobotLog.ii(TAG, "pursuit point=(%.1f, %.1f), robot rel=(%.1f, %.1f)",
                    pursuit.x, pursuit.y,
                    robotRelPursuitPoint.x, robotRelPursuitPoint.y);

            RobotLog.ii(TAG, "linear velocity=(%.1f, %.1f), robot rel=(%.1f, %.1f)",
                    fieldVel.x, fieldVel.y,
                    robotRelVel.x, robotRelVel.y);

            RobotLog.ii(TAG, "drive power: axial=%.3f, lateral=%.3f, yaw=%.3f",
                    axial.power, 0, 0);
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
2
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