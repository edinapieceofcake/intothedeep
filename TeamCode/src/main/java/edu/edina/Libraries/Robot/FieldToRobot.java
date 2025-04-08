package edu.edina.Libraries.Robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

public class FieldToRobot {
    public static Vector2d toRobotRel(Pose2d pose, Vector2d v) {
        double dx = getDx(pose);
        double dy = getDy(pose);
        double tx = v.x - pose.position.x;
        double ty = v.y - pose.position.y;
        return new Vector2d((tx * dy) - (ty * dx), (tx * dx) + (ty * dy));
    }

    public static Vector2d rotateToRobotRel(Rotation2d heading, Vector2d v) {
        return toRobotRel(new Pose2d(new Vector2d(0, 0), heading), v);
    }

    public static Vector2d toFieldRel(Pose2d pose, Vector2d v) {
        double dx = getDx(pose);
        double dy = getDy(pose);
        double tx = dy * v.x + dx * v.y;
        double ty = -dx * v.x + dy * v.y;
        return new Vector2d(pose.position.x + tx, pose.position.y + ty);
    }

    public static Vector2d[] toFieldRel(Pose2d pose, Vector2d[] v) {
        Vector2d[] rr = new Vector2d[v.length];
        for (int i = 0; i < v.length; i++)
            rr[i] = toFieldRel(pose, v[i]);

        return rr;
    }

    private static double getDx(Pose2d pose) {
        return -Math.sin(pose.heading.toDouble());
    }

    private static double getDy(Pose2d pose) {
        return Math.cos(pose.heading.toDouble());
    }
}