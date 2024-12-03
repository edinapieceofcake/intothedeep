package edu.edina.Libraries.Robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class FieldToRobot {
    public static Vector2d toRobotRel(Pose2d pose, Vector2d v) {
        double dx = getDx(pose);
        double dy = getDy(pose);
        double tx = v.x - pose.position.x;
        double ty = v.y - pose.position.y;
        return new Vector2d((tx * dy) - (ty * dx), (tx * dx) + (ty * dy));
    }

//    public Vector2d toFieldRel(Pose2d pose, Vector2d v) {
//        double dx = getDx();
//        double dy = getDy();
//        double tx = dy * v.x + dx * v.y;
//        double ty = -dx * v.x + dy * v.y;
//        return new Vector2d(pose.position.x + tx, pose.position. + ty, a);
//    }

    private static double getDx(Pose2d pose) {
        return -Math.sin(pose.heading.toDouble());
    }

    private static double getDy(Pose2d pose) {
        return Math.cos(pose.heading.toDouble());
    }
}