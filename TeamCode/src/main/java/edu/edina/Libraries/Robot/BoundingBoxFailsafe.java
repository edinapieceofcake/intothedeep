package edu.edina.Libraries.Robot;

/*
 Coordinates

 _                  90 degrees
 \\                 ^
  \\                |
 O======O           +----> x = 0 also 0 degrees


  Wrist angle is measured relative to the arm.
 */

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BoundingBoxFailsafe {
    public static double WRIST_UP_POSITION_DEG = 100;
    public static double WRIST_DOWN_POSITION_DEG = 20;
    public static double LIFT_INCH_MULT = 1;
    public static double LIFT_ANGLE = 120;

    private final Wrist wrist;
    private final Arm arm;
    private final Lift lift;
    private final Slide slide;

    public BoundingBoxFailsafe(Wrist wrist, Arm arm, Lift lift, Slide slide) {
        this.wrist = wrist;
        this.arm = arm;
        this.lift = lift;
        this.slide = slide;
    }

    public void updateSensorsForTestEventuallyRemoveThisMethod(){
        slide.updateVoltage();
    }

    public void apply() {
    }

    public void addToTelemetry(Telemetry telemetry) {
        telemetry.addData("arm position", getArmPosition());
        telemetry.addData("lift position", getLiftPosition());
        telemetry.addData("slide position", getSlidePosition());
        telemetry.addData("wrist position", getWristPosDeg());
    }

    private double getArmPosition() {
        double p = arm.getCurrentPosition();
        return p / Arm.TICKS_PER_DEGREE - Arm.INITIAL_DEGREES_BELOW_HORIZONTAL;
    }

    private double getSlidePosition() {
        return slide.getPosition();
    }

    private double getLiftPosition() {
        return lift.getPosition() * LIFT_INCH_MULT;
    }

    private double getWristPosDeg() {
        double p = wrist.getEstimatedPosition();
        double deg = interp(p,
                Wrist.DOWN_POSITION, Wrist.UP_POSITION,
                WRIST_DOWN_POSITION_DEG, WRIST_UP_POSITION_DEG);
        return deg;
    }

    private static double interp(double x, double domMin, double domMax, double rangeMin, double rangeMax) {
        return (x - domMin) / (domMax - domMin) * (rangeMax - rangeMin) + rangeMin;
    }
}
