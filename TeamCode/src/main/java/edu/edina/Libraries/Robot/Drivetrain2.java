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
import edu.edina.Tests.PurePursuit.MotorCommand;

// This represents a drivetrain.
@Config
public class Drivetrain2 {
    public static double ALIGNMENT_NORM_MIN = 10;

    private ContinuousBooleanTest booleanTest;

    private final String tag = "Drivetrain";

    public static double VEC_TRACK_ANGLE = 30;
    public static double MAX_PURSUIT_INCHES = 10;
    public static double YAW_DEADZONE = 0.1;
    public static double HEADING_P_COEF = 1;

    public static int MILISECOND = 300;

    // Left back drive
    private final DcMotorEx leftBack;

    // Left front drive.
    private final DcMotorEx leftFront;

    // Right back drive
    private final DcMotorEx rightBack;

    // Right front drive
    private final DcMotorEx rightFront;

    private DcMotorEx[] motors;

    private final RobotState robotState;

    public Drivetrain2(HardwareMap hw, RobotState robotState) {
        this.robotState = robotState;
        booleanTest = new ContinuousBooleanTest(MILISECOND);

        // Get the hardware map.
        // Get the motors.
        leftBack = hw.get(DcMotorEx.class, "left_back_drive");
        leftFront = hw.get(DcMotorEx.class, "left_front_drive");
        rightBack = hw.get(DcMotorEx.class, "right_back_drive");
        rightFront = hw.get(DcMotorEx.class, "right_front_drive");

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

        currPose = robotState.getCurrentPose();
        refPose = currPose;
    }

    private Pose2d currPose;
    private Pose2d refPose;

    public void update2(Gamepad gamepad1, Gamepad gamepad2, double playerTwoMult) {
        if (gamepad1.left_trigger > 0.6) {
            for (DcMotorEx motor : motors) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        } else {
            for (DcMotorEx motor : motors) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }

        double gamepadYaw = gamepad1.right_stick_x + gamepad2.right_stick_x * playerTwoMult;
        double gamepadLateral = -(gamepad1.left_stick_x + gamepad2.left_stick_x * playerTwoMult);
        double gamepadAxial = -(gamepad1.left_stick_y + gamepad2.left_stick_y * playerTwoMult);

        Pose2dDual<Time> poseDual = robotState.getCurrentPoseDual();

        currPose = poseDual.value();

        Vector2d vel = poseDual.position.drop(1).value();

        RobotLog.ii(tag, "vel = (%.1f, %.1f)", vel.x, vel.y);

        Vector2d c = new Vector2d(gamepadAxial, gamepadLateral);

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
        boolean nowInDeadZone = Math.abs(gamepadYaw) < YAW_DEADZONE;
        if (booleanTest.update(nowInDeadZone)) {
            // set the power using PID
            double radDiff = Angle.radianDiff(currPose.heading.toDouble(), refPose.heading.toDouble());
            yawPower = HEADING_P_COEF * radDiff;

            // update reference heading
            double norm = vel.norm();
            if (gamepad1.y && norm > ALIGNMENT_NORM_MIN) {
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
            yawPower = gamepadYaw;

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

        // use MotorCommand here...
        if (r + a > 1) {

        }

        update(axial, lateral, yaw);
    }

    public void update(double axial, double lateral, double yaw) {
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

        // Move the robot.
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
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
