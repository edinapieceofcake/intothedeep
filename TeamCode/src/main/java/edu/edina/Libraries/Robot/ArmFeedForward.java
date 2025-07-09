package edu.edina.Libraries.Robot;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.util.RobotLog;

import edu.edina.Libraries.LinearMotion.IFeedForward;

public class ArmFeedForward implements IFeedForward {
    private RobotState rs;

    public ArmFeedForward(RobotState rs) {
        this.rs = rs;
    }

    @Override
    public double getPower(DualNum<Time> armPosVel) {
        double extPos = 0.85 * rs.getExtensionPos() + 2;
        double armPos = rs.getArmPos() - Arm.POS_ARM_WALL;
        double horizontalFeedforward = extPos * Arm.MAX_FEED_FWD_MULT;
        double angleMult = Math.cos(Math.toRadians(armPos));

        return horizontalFeedforward * angleMult;
    }
}
