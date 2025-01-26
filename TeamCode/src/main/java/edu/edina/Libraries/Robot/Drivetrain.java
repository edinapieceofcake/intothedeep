package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// This represents a drivetrain.
@Config
public class Drivetrain {

    // Normal multiplier
    public static double NORMAL_MULTIPLIER = 1;

    // Turtle multiplier
    public static double TURTLE_MULTIPLIER = 0.3;

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

    // Initializes this.
    public Drivetrain(LinearOpMode opMode) {

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
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Updates this.
    public void update() {
        double m = drivingReverseMult();

        Gamepad gamepad1 = opMode.gamepad1;
        Gamepad gamepad2 = opMode.gamepad2;
        Gamepad currentGamepad;

        boolean isDriver1InControl = Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.right_stick_x) > 0.1;

        if (isDriver1InControl) {
            currentGamepad = gamepad1;
        }
        else {
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

        double multiplier = getTurtleMode() || !isDriver1InControl ? TURTLE_MULTIPLIER : NORMAL_MULTIPLIER;

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