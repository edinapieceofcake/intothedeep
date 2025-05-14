package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.edina.Libraries.Actions.ContinuousBooleanTest;
import edu.edina.Libraries.Angle;
import edu.edina.Libraries.VectorCalc;

// This represents a drivetrain.
@Config
public class Drivetrain {
    public static double ALIGNMENT_NORM_MIN = 10;

    private ContinuousBooleanTest booleanTest;

    // Normal multiplier
    public static double NORMAL_MULTIPLIER = 1;
    public static double YAW_PRIORITY = 0.5;

    private final String tag = "Drivetrain";

    public static double VEC_TRACK_ANGLE = 30;
    public static double MAX_PURSUIT_INCHES = 10;
    public static double YAW_DEADZONE = 0.1;
    public static double HEADING_P_COEF = 2;

    public static int MILISECOND = 300;

    // Turtle multiplier
    public static double SNAIL_MULTIPLIER = 0.3;
    public static double TURTLE_MULTIPLIER = 0.55;

    public final static boolean ENABLE_REVERSE = true;

    // Left back drive
    private final DcMotorEx leftBack;

    // Left front drive.
    private final DcMotorEx leftFront;

    // Right back drive
    private final DcMotorEx rightBack;

    // Right front drive
    private final DcMotorEx rightFront;

    // Op mode
    private LinearOpMode opMode;

    // Turtle mode
    private boolean turtleMode, autoTurtleMode;

    private boolean reversed;

    private DcMotorEx[] motors;

    private OpticalOdometry odo;

    // Initializes this.
    public Drivetrain(LinearOpMode opMode) {
        booleanTest = new ContinuousBooleanTest(MILISECOND);

        // Remember the op mode.
        this.opMode = opMode;

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Get the motors.
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front_drive");

        // Set the motor directions.
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        // Set the motor zero power behavior.
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motors = new DcMotorEx[]{leftBack, leftFront, rightBack, rightFront};

        odo = new OpticalOdometry(hardwareMap);
        currPose = odo.getCurrentPose();
        refPose = currPose;
    }

    private Pose2d currPose;
    private Pose2d refPose;

    public void update2() {
        if (opMode.gamepad1.left_trigger > 0.6) {
            for (DcMotorEx motor : motors) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        } else {
            for (DcMotorEx motor : motors) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }

        Pose2dDual<Time> poseDual = odo.getCurrentPoseDual();

        currPose = poseDual.value();

        Vector2d vel = poseDual.position.drop(1).value();

        RobotLog.ii(tag, "vel = (%.1f, %.1f)", vel.x, vel.y);

        Vector2d c = new Vector2d(-opMode.gamepad1.left_stick_y, -opMode.gamepad1.left_stick_x);

        Vector2d newRefPos;
        if (VectorCalc.angleBetweenDeg(c, vel) < VEC_TRACK_ANGLE) {
            Vector2d p1 = currPose.position;
            Vector2d q0 = refPose.position;
            newRefPos = VectorCalc.project(p1.minus(q0), c).plus(q0);
        } else {
            newRefPos = currPose.position;
        }

        double yawPower;
        Rotation2d newRefHead;

        //if turning robot change heading and update target
        //if not pushing stick then maintain target heading
        boolean nowInDeadZone = Math.abs(opMode.gamepad1.right_stick_x) < YAW_DEADZONE;
        if (booleanTest.update(nowInDeadZone)) {
            // set the power using PID
            double radDiff = Angle.radianDiff(currPose.heading.toDouble(), refPose.heading.toDouble());
            yawPower = HEADING_P_COEF * radDiff;

            // update reference heading
            double norm = vel.norm();
            if (opMode.gamepad1.b && norm > ALIGNMENT_NORM_MIN) {
                Vector2d unitVel = vel.div(norm);
                newRefHead = new Rotation2d(unitVel.x, unitVel.y);

                if (Math.abs(Angle.radianDiff(newRefHead, currPose.heading)) > (Math.PI / 2)) {
                    newRefHead = newRefHead.plus(Math.PI);
                }
            } else {
                newRefHead = refPose.heading;
            }
        } else {
            // set the power manually
            yawPower = opMode.gamepad1.right_stick_x;

            // current heading *becomes* the reference heading
            newRefHead = currPose.heading;
        }

        RobotLog.ii(tag, "ref heading = %.1f act heading = %.1f",
                Math.toDegrees(newRefHead.toDouble()),
                Math.toDegrees(currPose.heading.toDouble())
        );

        refPose = new Pose2d(newRefPos, newRefHead);

        Vector2d pursuitPoint = refPose.position.plus(c.times(MAX_PURSUIT_INCHES));

        Vector2d relPursuitPoint = FieldToRobot.toRobotRel(currPose, pursuitPoint);
        double relNorm = relPursuitPoint.norm();
        if (relNorm > 0)
            relPursuitPoint = relPursuitPoint.div(relNorm);
        Vector2d drivePower = relPursuitPoint.times(c.norm());

        double axial = drivePower.x;
        double lateral = -drivePower.y;
        double yaw = yawPower < -1 ? -1 : (yawPower > 1 ? 1 : yawPower);

        RobotLog.ii(tag, "powers axial = %.2f, lateral = %.2f, yaw = %.2f, c.x = %.2f, c.y = %.2f",
                axial, lateral, yaw, c.x, c.y);

        double r = Math.max(Math.abs(axial + lateral), Math.abs(axial - lateral));
        double a = Math.abs(yaw);

        if (r + a > 1) {

        }

        update(axial, lateral, yaw);
    }

    // Updates this.
    public void update() {
        double m = drivingReverseMult();

        Gamepad gamepad1 = opMode.gamepad1;
        Gamepad gamepad2 = opMode.gamepad2;
        Gamepad currentGamepad;

        boolean isDriver1InControl = Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.right_stick_x) > 0.1;

        if (gamepad1.left_trigger > 0.6) {
            for (DcMotorEx motor : motors) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        } else {
            for (DcMotorEx motor : motors) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }

        if (isDriver1InControl) {
            currentGamepad = gamepad1;
        } else {
            currentGamepad = gamepad2;
        }

        double axial = -currentGamepad.left_stick_y * m;  // Note: pushing stick forward gives negative value
        double lateral = currentGamepad.left_stick_x * m;
        double yaw = currentGamepad.right_stick_x;

        update(axial, lateral, yaw, isDriver1InControl);

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Display arm telemetry.
        telemetry.addData("Drivetrain", "====================");
        telemetry.addData("- Turtle Mode", turtleMode);
    }

    public void update(double axial, double lateral, double yaw, boolean isDriver1InControl) {
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        double multiplier;

        if (isDriver1InControl) {
            multiplier = getTurtleMode() ? TURTLE_MULTIPLIER : NORMAL_MULTIPLIER;
        } else {
            multiplier = SNAIL_MULTIPLIER;
        }
        leftFrontPower *= multiplier;
        leftBackPower *= multiplier;
        rightBackPower *= multiplier;
        rightFrontPower *= multiplier;

        // Move the robot.
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }

    public void update(double axial, double lateral, double yaw) {
        update(axial, lateral, yaw, true);
    }

    // Sets the turtle mode value.
    public void setTurtleMode(boolean turtleMode) {

        // Set the turtle mode value.
        this.turtleMode = turtleMode;

    }

    public void setAutoTurtleMode(boolean autoTurtleMode) {
        this.autoTurtleMode = autoTurtleMode;
    }

    private boolean getTurtleMode() {
        return turtleMode || autoTurtleMode;
    }

    public void setReverse(boolean reverse) {
        reversed = reverse;
    }

    private double drivingReverseMult() {
        if (reversed && ENABLE_REVERSE)
            return -1;
        else
            return 1;
    }

    // Stops the robot.
    public void stop() {

        // Stop the robot.
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

    }
}