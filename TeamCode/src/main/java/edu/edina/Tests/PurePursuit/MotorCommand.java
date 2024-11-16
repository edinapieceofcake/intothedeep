package edu.edina.Tests.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import kotlin.NotImplementedError;

@Disabled
public class MotorCommand {
    public MotorCommand(double axial, double lateral, double yaw) {
        leftFrontPower = axial - lateral + yaw;
        rightFrontPower = axial + lateral - yaw;
        leftBackPower = axial + lateral + yaw;
        rightBackPower = axial - lateral - yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
    }

    private double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;

    public void scale(double x) {
        MotorCommand mc;

        if (x >= -1 && x <= 1) {
            leftFrontPower *= x;
            rightFrontPower *= x;
            leftBackPower *= x;
            rightBackPower *= x;
        }
    }

    public double getLeftFrontPower() {
        return leftFrontPower;
    }

    public double getRightFrontPower() {
        return rightFrontPower;
    }

    public double getLeftBackPower() {
        return leftBackPower;
    }

    public double getRightBackPower() {
        return rightBackPower;
    }

    @Override
    public String toString() {
        return String.format("%.4f, %.4f, %.4f, %.4f", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }
}
