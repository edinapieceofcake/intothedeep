package edu.edina.Libraries.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.edina.Libraries.PurePursuit.PurePursuit;

@Config
public class SpecimenPark implements Action {
    public static double M = 0.1;
    public static double N = 0.1;
    private final RobotHardware hw;
    private final PurePursuit pursuit;
    public static double radius = 2;
    private final Telemetry telemetry;
    private final Odometry odometry;
    private final double finalHeading;

    public SpecimenPark(RobotHardware hw) {
        this.hw = hw;
        telemetry = hw.getOpMode().telemetry;

        double left = hw.distanceSensors.readLeftBack();
        double right = hw.distanceSensors.readRightBack();
        telemetry.addData("left", left);
        telemetry.addData("right", right);
        telemetry.update();

        Vector2d[] robotCentricPath = new Vector2d[]{
                new Vector2d(0, 0),
                pursuit(left, right, false),
                pursuit(left, right, true)
        };

        odometry = hw.getOdometry();
        Pose2d pose = odometry.getPoseEstimate();

        Vector2d[] fieldCentricPath = FieldToRobot.toFieldRel(pose, robotCentricPath);

        finalHeading = Math.atan2(
                fieldCentricPath[1].y - fieldCentricPath[2].y,
                fieldCentricPath[1].x - fieldCentricPath[2].x
        );

        RobotLog.ii("SpecimenPark", "finalHeading: %.4f (rad), %.1f (deg)",
            finalHeading, Math.toDegrees(finalHeading));

        pursuit = new PurePursuit(fieldCentricPath, false);

        for (Vector2d v : pursuit.getPath()) {
            RobotLog.ii("SpecimenPark", "path: x = %.2f y = %.2f", v.x, v.y);
        }
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
        Pose2d currentPos = odometry.getPoseEstimate();

        pursuit.nextPursuitPoint(currentPos.position, radius);

        telemetry.addData("pursuit point", "x = %.2f y = %.2f", pursuit.getPursuitPoint().x, pursuit.getPursuitPoint().y);

        Vector2d fcPursuitPoint = pursuit.getPursuitPoint();
        Vector2d pursuitPoint = FieldToRobot.toRobotRel(currentPos, fcPursuitPoint);

        double axial = M * pursuitPoint.x;
        double lateral = M * pursuitPoint.y;

        double rotation = N * (finalHeading - currentPos.heading.toDouble());

        hw.drivetrain.update(axial, -lateral, -rotation);

        RobotLog.ii("SpecimenPark", "pursuit point: x = %.2f y = %.2f (fc)", fcPursuitPoint.x, fcPursuitPoint.y);
        RobotLog.ii("SpecimenPark", "pursuit point: x = %.2f y = %.2f (rr)", pursuitPoint.x, pursuitPoint.y);
        RobotLog.ii("SpecimenPark", "alpha = %.2f (rad)", (rotation / N));
        RobotLog.ii("SpecimenPark", "power: axial = %.2f lateral = %.2f rotation = %.2f", axial, lateral, rotation);

        return true;
    }
}