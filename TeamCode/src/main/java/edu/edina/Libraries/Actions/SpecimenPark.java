package edu.edina.Libraries.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.edina.Libraries.MotionControl.ILinearMechanism;
import edu.edina.Libraries.LinearMotion.LinearMechanismSettings;
import edu.edina.Libraries.LinearMotion.LinearMotionController;
import edu.edina.Libraries.LinearMotion.RotationalDriveMechanism;
import edu.edina.Libraries.PurePursuit.PurePursuit;
import edu.edina.Libraries.Robot.FieldToRobot;
import edu.edina.Libraries.Robot.Odometry;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.SensorLayout;

@Config
@Controls(part = "drivetrain")
public class SpecimenPark implements Action {
    public static double M = 0.1;
    private final RobotHardware hw;
    private final PurePursuit pursuit;
    public static double radius = 2;
    private final Telemetry telemetry;
    private final Condition condition;
    private final Odometry odometry;

    // motion control
    private final RotationalMechanism rotMech;
    private final LinearMotionController rotCon;

    private final boolean sanityCheck;

    public SpecimenPark(RobotHardware hw) {
        this(hw, null);
    }

    public SpecimenPark(RobotHardware hw, Condition condition) {
        this.hw = hw;
        telemetry = hw.getOpMode().telemetry;
        this.condition = condition;

        double left = hw.distanceSensors.readLeftBack();
        double right = hw.distanceSensors.readRightBack();

        sanityCheck = !(left > 28) && !(right > 28) && !(Math.abs(left - right) > 4);

        RobotLog.ii("SpecimenPark", "init -- distance left %.1f right %.1f",
                left, right);

        Vector2d[] robotCentricPath = new Vector2d[]{
                new Vector2d(0, 0),
                pursuit(left, right, false),
                pursuit(left, right, true)
        };

        odometry = hw.getOdometry();
        Pose2d pose = odometry.getPoseEstimate();

        Vector2d[] fieldCentricPath = FieldToRobot.toFieldRel(pose, robotCentricPath);

        double targetHeading = Math.toDegrees(Math.atan2(
                fieldCentricPath[1].y - fieldCentricPath[2].y,
                fieldCentricPath[1].x - fieldCentricPath[2].x
        ));

        RobotLog.ii("SpecimenPark", "init -- heading: %.1f (deg) targetHeading: %.1f (deg)",
                Math.toDegrees(pose.heading.toDouble()), targetHeading);

        pursuit = new PurePursuit(fieldCentricPath, false);

        for (Vector2d v : pursuit.getPath()) {
            RobotLog.ii("SpecimenPark", "init -- path: x = %.2f y = %.2f", v.x, v.y);
        }

        rotMech = new RotationalMechanism();
        rotCon = new LinearMotionController(rotMech);
        rotCon.setTarget(targetHeading);
    }

    public static Vector2d pursuit(double left, double right, boolean near) {
        double r1 = -left + SensorLayout.centerOffset;
        double r2 = -right + SensorLayout.centerOffset;
        double avg = (r1 + r2) / 2;
        double delta = r2 - r1;
        double angle = Math.atan2(delta, SensorLayout.width);

        double distance;
        if (near) distance = 10;
        else distance = 13;

        return calcVec(avg, angle, -distance);
    }

    private static Vector2d calcVec(double avg, double angle, double distance) {
        double x = avg - distance * Math.cos(angle);
        double y = -distance * Math.sin(angle);
        return new Vector2d(x, y);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (condition != null && !condition.run())
            return false;

        if (!sanityCheck)
            return false;

        Pose2d currentPos = odometry.getPoseEstimate();

        pursuit.calcNextPursuitPoint(currentPos.position, radius);

        telemetry.addData("pursuit point", "x = %.2f y = %.2f", pursuit.getPursuitPoint().x, pursuit.getPursuitPoint().y);

        Vector2d fcPursuitPoint = pursuit.getPursuitPoint();
        Vector2d pursuitPoint = FieldToRobot.toRobotRel(currentPos, fcPursuitPoint);

        double axial = M * pursuitPoint.x;
        double lateral = M * pursuitPoint.y;

        boolean doneRotating = rotCon.run();

        double rotPow = rotMech.power;

        hw.drivetrain.update(axial, -lateral, rotPow);

        RobotLog.ii("SpecimenPark", "pursuit point: x = %.2f y = %.2f (fc)", fcPursuitPoint.x, fcPursuitPoint.y);
        RobotLog.ii("SpecimenPark", "pursuit point: x = %.2f y = %.2f (rr)", pursuitPoint.x, pursuitPoint.y);
        RobotLog.ii("SpecimenPark", "power: axial = %.2f lateral = %.2f rotation = %.2f", axial, lateral, rotPow);

        return !doneRotating;
    }

    private class RotationalMechanism implements ILinearMechanism {
        private double power;

        @Override
        public void setPower(double power) {
            this.power = -power;
        }

        @Override
        public double getPosition(boolean raw) {
            return getPositionAndVelocity(raw).get(0);
        }

        @Override
        public DualNum<Time> getPositionAndVelocity(boolean raw) {
            Pose2d p = odometry.getPoseEstimate();
            PoseVelocity2d v = odometry.getVelocityEstimate();
            return new DualNum<>(new double[]{Math.toDegrees(p.heading.toDouble()), Math.toDegrees(v.angVel)});
        }

        @Override
        public LinearMechanismSettings getSettings() {
            return RotationalDriveMechanism.getStaticSettings();
        }
    }
}