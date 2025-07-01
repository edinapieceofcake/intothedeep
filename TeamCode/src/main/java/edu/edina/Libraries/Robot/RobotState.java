package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import edu.edina.Libraries.MovingAverageCalc;

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
    private final ElapsedTime t;
    private final MovingAverageCalc voltAvg;

    private final BackgroundSensorReader<BackgroundData> bgData;

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

        extensionSpeed = new Speedometer(5);
        armSpeed = new Speedometer(3);
        leftSpeed = new Speedometer(3);
        rightSpeed = new Speedometer(3);

        t = new ElapsedTime();
        voltAvg = new MovingAverageCalc(3);

        bgData = new BackgroundSensorReader<>(() -> readBackgroundData(), 1);
    }

    public void update(Telemetry telemetry) {
        BackgroundData bgData = this.bgData.getValue();
        extensionPos = extensionMotor.getCurrentPosition();
        extensionSpeed.sample(extensionPos);

        armPos = armMotor.getCurrentPosition();
        armSpeed.sample(armPos);

        leftPos = leftMotor.getCurrentPosition();
        leftSpeed.sample(leftPos);

        rightPos = rightMotor.getCurrentPosition();
        rightSpeed.sample(rightPos);

        voltage = bgData.voltage;
        voltAvg.update(t.seconds(), voltage);

        poseDual = bgData.poseDual;

        frontDist = bgData.frontDist;
        leftDist = bgData.leftDist;
        rightDist = bgData.rightDist;

        Pose2d p = getCurrentPose();
        telemetry.addData("v_avg", "%.2fV", voltAvg.getAverage());
        telemetry.addData("pose", "(%.1f, %.1f) %.1f deg", p.position.x, p.position.y, Math.toDegrees(p.heading.toDouble()));
        telemetry.addData("lift", "%.1f(%.1f, %.1f) in   %.1f in/s",
                getLiftPos(), getLeftLiftPos(), getRightLiftPos(), getLiftSpeed());
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

    public double getAverageVoltage() {
        return voltAvg.getAverage();
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

    public void calibrateIMU() {
        odo.calibrateIMU();
    }

    private static class BackgroundData {
        public final double voltage, leftDist, rightDist, frontDist;
        public final Pose2dDual<Time> poseDual;

        private BackgroundData(double voltage, double leftDist, double rightDist, double frontDist, Pose2dDual<Time> poseDual) {
            this.voltage = voltage;
            this.leftDist = leftDist;
            this.rightDist = rightDist;
            this.frontDist = frontDist;
            this.poseDual = poseDual;
        }
    }

    private BackgroundData readBackgroundData() {
        double leftDist = leftDistance.getDistance(DistanceUnit.INCH);
        double rightDist = rightDistance.getDistance(DistanceUnit.INCH);
        double frontDist = frontDistance.getDistance(DistanceUnit.INCH);
        double voltage = vs.getVoltage();
        Pose2dDual<Time> poseDual = odo.getCurrentPoseDual();

        return new BackgroundData(voltage, leftDist, rightDist, frontDist, poseDual);
    }
}