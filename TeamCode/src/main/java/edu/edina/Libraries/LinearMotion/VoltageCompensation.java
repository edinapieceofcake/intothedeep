package edu.edina.Libraries.LinearMotion;

import edu.edina.Libraries.Robot.RobotState;

public class VoltageCompensation {
    private final RobotState robotState;

    public VoltageCompensation(RobotState robotState) {
        this.robotState = robotState;
    }

    public double adjustPower(double power) {
        double v = robotState.getAverageVoltage();
        double multiplier = 12 / v;
        return power * multiplier;
    }
}
