package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import edu.edina.Libraries.LinearMotion.ILinearMechanism;
import edu.edina.Libraries.LinearMotion.LinearMechanismSettings;

@Config
public class MecanumLinearMechanism implements ILinearMechanism {
    private static String TAG = "MecanumLinearMechanism";

    public static double AXIAL = 1;
    public static double LATERAL = 1;

    public static double KS;
    public static double KV;
    public static double KA;
    public static double ACCEL_CAL_DIST;

    private final DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private final OpticalOdometry odo;

    public MecanumLinearMechanism(HardwareMap hardwareMap) {
        odo = new OpticalOdometry(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back_drive");

        // Set the motor directions.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void setPower(double power) {
        leftFront.setPower(power * (AXIAL + LATERAL));
        leftBack.setPower(power * (AXIAL - LATERAL));
        rightFront.setPower(power * (AXIAL - LATERAL));
        rightBack.setPower(power * (AXIAL + LATERAL));
    }

    @Override
    public double getPosition(boolean raw) {
        return getPositionAndVelocity(raw).value();
    }

    @Override
    public DualNum<Time> getPositionAndVelocity(boolean raw) {
        Pose2dDual<Time> pva = odo.getCurrentPoseDual();
        Vector2d p = pva.position.value();
        double a = Math.toDegrees(Math.atan2(p.y, p.x));
        RobotLog.ii(TAG, "position (%.2f, %.2f) / angle %.1f", p.x, p.y, a);
        return new DualNum<>(new double[]{p.norm(), pva.position.drop(1).value().norm()});
    }

    @Override
    public LinearMechanismSettings getSettings() {
        return new LinearMechanismSettings(KS, KV, KA, ACCEL_CAL_DIST);
    }
}
