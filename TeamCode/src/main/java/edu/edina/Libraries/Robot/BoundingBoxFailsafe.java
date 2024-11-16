package edu.edina.Libraries.Robot;

/*
 Coordinates

 _                  90 degrees
 \\                 ^
  \\                |
 O======O           +----> x = 0 also 0 degrees


  Wrist angle is measured relative to the arm.
 */

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.libuvc.api.UvcApiExposureControl;

@Config
public class BoundingBoxFailsafe {
    // DO NOT DISABLE IN CODE - only disable this for testing from the dashboard
    public static boolean DISABLE = false;

    private static double WRIST_UP_POSITION_DEG = 80;
    private static double WRIST_DOWN_POSITION_DEG = 25;
    private static double LIFT_ANGLE = 180 - 72;
    private static double WRIST_HEEL_X = 0.75;
    private static double WRIST_TIP_X = 6.75;
    private static double WRIST_Y = 2.125;
    private static double MIN_SLIDE_LENGTH = 0;

    public static double BOUNDING_BOX_MAX = 19;
    public static double BOUNDING_BOX_MIN = BOUNDING_BOX_MAX - 42;

    private final Wrist wrist;
    private final Arm arm;
    private final Lift lift;
    private final Slide slide;

    private double minExtent, maxExtent;

    public BoundingBoxFailsafe(Wrist wrist, Arm arm, Lift lift, Slide slide) {
        this.wrist = wrist;
        this.arm = arm;
        this.lift = lift;
        this.slide = slide;
    }

    // use a globally-updated sensor caching mechanism instead, eventually
    public void updateSensorsForTestEventuallyRemoveThisMethod() {
        slide.updateVoltage();
    }

    public void apply() {
        if (DISABLE)
            return;

        Pos p = new Pos();

        double hx = estimateWristExtent(WRIST_HEEL_X, p);
        double tx = estimateWristExtent(WRIST_TIP_X, p);

        minExtent = Math.min(hx, tx);
        maxExtent = Math.max(hx, tx);

        if (minExtent < BOUNDING_BOX_MIN) {
            if (p.armDeg < 170) {
                arm.overridePower(-1);
            }

            wrist.raise();
            slide.overridePower(-1);
        } else if (maxExtent > BOUNDING_BOX_MAX) {
            if (p.armDeg > 0) {
                arm.overridePower(1);
            } else {
                arm.overridePower(-1);
            }

            wrist.raise();
            slide.overridePower(-1);
        }
    }

    public void addToTelemetry(Telemetry telemetry) {
        telemetry.addData("arm position", getArmPosition());
        telemetry.addData("lift position", getLiftPosition());
        telemetry.addData("slide position", getSlidePosition());
        telemetry.addData("wrist position", getWristPosDeg());
        telemetry.addData("min extent", minExtent);
        telemetry.addData("max extent", maxExtent);
    }

    private double estimateWristExtent(double wristX, Pos p) {
        // transform the axle point
        Vector2d armAxlePt = new Vector2d(0, p.liftY);
        armAxlePt = rotate(armAxlePt, LIFT_ANGLE);

        // transform the wrist point
        Vector2d wristPt = new Vector2d(wristX, WRIST_Y);
        wristPt = rotate(wristPt, p.wristDeg);
        wristPt = translateX(wristPt, p.slideX);
        wristPt = rotate(wristPt, p.armDeg);
        wristPt = translate(wristPt, armAxlePt);

        return wristPt.x;
    }

    private double getArmPosition() {
        double p = arm.getCurrentPosition();
        return p / Arm.TICKS_PER_DEGREE - Arm.INITIAL_DEGREES_BELOW_HORIZONTAL;
    }

    private double getSlidePosition() {
        return slide.getPosition() + MIN_SLIDE_LENGTH;
    }

    private double getLiftPosition() {
        return lift.getPositionInInches();
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

    private static Vector2d rotate(Vector2d v, double degrees) {
        double theta = Math.toRadians(degrees);
        double c = Math.cos(theta);
        double s = Math.sin(theta);
        return new Vector2d(
                c * v.x - s * v.y,
                s * v.x + c * v.y);
    }

    private static Vector2d translateX(Vector2d v, double x) {
        return new Vector2d(v.x + x, v.y);
    }

    private static Vector2d translate(Vector2d v, Vector2d u) {
        return new Vector2d(v.x + u.x, v.y + u.y);
    }

    private class Pos {
        public double armDeg, liftY, slideX, wristDeg;

        public Pos() {
            armDeg = getArmPosition();
            liftY = getLiftPosition();
            slideX = getSlidePosition();
            wristDeg = getWristPosDeg();
        }
    }
}
