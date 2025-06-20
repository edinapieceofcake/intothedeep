package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class RobotState {
    private VoltageSensor vs;
    private DcMotorEx extensionMotor, armMotor, leftMotor, rightMotor;
    private OpticalOdometry odo;
    private Rev2mDistanceSensor leftDistance, rightDistance, frontDistance;

    private int extensionPos, armPos, leftPos, rightPos;
    private Speedometer extensionSpeed, armSpeed, leftSpeed, rightSpeed;
    private double voltage, leftDist, rightDist, frontDist;
    private Pose2dDual<Time> poseDual;

    public RobotState(HardwareMap hw) {
        leftDistance = hw.get(Rev2mDistanceSensor.class, "distance_left");
        rightDistance = hw.get(Rev2mDistanceSensor.class, "distance_right");
        frontDistance = hw.get(Rev2mDistanceSensor.class, "distance_front");
        extensionMotor = hw.get(DcMotorEx.class, "extension_motor");
        armMotor = hw.get(DcMotorEx.class, "arm_motor");
        leftMotor = hw.get(DcMotorEx.class, "left_lift_motor");
        rightMotor = hw.get(DcMotorEx.class, "right_lift_motor");

        extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        vs = hw.voltageSensor.iterator().next();
        odo = new OpticalOdometry(hw);

        poseDual = odo.getCurrentPoseDual();

        extensionSpeed = new Speedometer(3);
        armSpeed = new Speedometer(3);
        leftSpeed = new Speedometer(3);
        rightSpeed = new Speedometer(3);
    }

    public void update(Telemetry telemetry) {
        leftDist = leftDistance.getDistance(DistanceUnit.INCH);
        rightDist = rightDistance.getDistance(DistanceUnit.INCH);
        frontDist = frontDistance.getDistance(DistanceUnit.INCH);

        extensionPos = extensionMotor.getCurrentPosition();
        extensionSpeed.sample(extensionPos);

        armPos = armMotor.getCurrentPosition();
        armSpeed.sample(armPos);

        leftPos = leftMotor.getCurrentPosition();
        leftSpeed.sample(leftPos);

        rightPos = rightMotor.getCurrentPosition();
        rightSpeed.sample(rightPos);

        voltage = vs.getVoltage();

        poseDual = odo.getCurrentPoseDual();

        Pose2d p = getCurrentPose();
        telemetry.addData("pose", "(%.1f, %.1f) %.1f deg", p.position.x, p.position.y, Math.toDegrees(p.heading.toDouble()));
        telemetry.addData("lift", "%.1f in   %.1f in/s", getLiftPos(), getLiftSpeed());
        telemetry.addData("arm", "%.1f deg   %.1f deg/s", getArmPos(), getArmSpeed());
        telemetry.addData("ext", "%.1f in   %.1f in/s", getExtensionPos(), getExtensionSpeed());
        telemetry.addData("dist", "%.1f in (f) %.1f in (l) %.1f (r)", frontDist, leftDist, rightDist);
    }

    public double getRightLiftPos() {
        return rightPos * Lift2.LIFT_MULT;
    }

    public double getRightLiftSpeed() {
        return rightSpeed.getSpeed() * Lift2.LIFT_MULT;
    }

    public double getLeftLiftPos() {
        return leftPos * Lift2.LIFT_MULT;
    }

    public double getLeftLiftSpeed() {
        return leftSpeed.getSpeed() * Lift2.LIFT_MULT;
    }

    public double getLiftPos() {
        return (getLeftLiftPos() - getRightLiftPos()) / 2.0;
    }

    public double getLiftSpeed() {
        return (getLeftLiftSpeed() - getRightLiftSpeed()) / 2.0;
    }

    public double getArmPos() {
        return armPos * (180.0 / Arm2.POS_AT_180_DEG_ARM);
    }

    public double getArmSpeed() {
        return armSpeed.getSpeed() * (180.0 / Arm2.POS_AT_180_DEG_ARM);
    }

    public double getExtensionPos() {
        return extensionPos * Extension.EXTENSION_MULT;
    }

    public double getExtensionSpeed() {
        return extensionSpeed.getSpeed() * Extension.EXTENSION_MULT;
    }

    public double getVoltage() {
        return voltage;
    }

    public Pose2d getCurrentPose() {
        return poseDual.value();
    }

    public Pose2dDual<Time> getCurrentPoseDual() {
        return poseDual;
    }

    public double getLeftDist() {
        return leftDist;
    }

    public double getRightDist() {
        return rightDist;
    }

    public double getFrontDist() {
        return frontDist;
    }

    public boolean armOverSub() {
        return getArmPos() > 180;
    }
}