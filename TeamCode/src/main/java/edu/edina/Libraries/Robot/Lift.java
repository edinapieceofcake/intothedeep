package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// This represents a lift.
@Config
public class Lift {

    // Derivative coefficient
    public static double DERIVATIVE = 0.0001;

    // Feedforward coefficient
    public static double FEEDFORWARD = 0.1;

    // High baskket position
    public static int HIGH_BASKET_POSITION = 1800;

    // Integral coefficient
    public static double INTEGRAL = 0;

    // Maximum position
    public static int MAXIMUM_POSITION = 1800;

    // Minimum position
    public static int MINIMUM_POSITION = 0;

    // Position increment
    public static int POSITION_INCREMENT = 50;

    // Proportional coefficient
    public static double PROPORTIONAL = 0.002;

    // Threshold
    public static int THRESHOLD = 50;

    // Controller
    private PIDController controller;

    // Left motor
    private final DcMotorEx leftMotor;

    // Right motor
    private final DcMotorEx rightMotor;

    // Robot hardware
    private RobotHardware robotHardware;

    // Target position
    private int targetPosition;

    // Touch sensor
    private final TouchSensor touch;

    // Initializes this.
    public Lift(RobotHardware robotHardware) {

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Get the left motor.
        leftMotor = hardwareMap.get(DcMotorEx.class, "left_lift_motor");

        // Set the left motor's direction.
        leftMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Get the right motor.
        rightMotor = hardwareMap.get(DcMotorEx.class, "right_lift_motor");

        // Set the right motor's direction.
        rightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        // Reset the motors.
        reset();

        // Get the touch sensor.
        touch = hardwareMap.get(TouchSensor.class, "lift_touch");

        // Initialize the lift controller.
        controller = new PIDController(PROPORTIONAL, INTEGRAL, DERIVATIVE);

    }

    // Update this.
    public void update() {

        // Update the lift controller.
        //////////////////////////////////////////////////////////////////////

        // PIDF Loops & Arm Control | FTC | 16379 KookyBotz
        // https://www.youtube.com/watch?v=E6H6Nqe6qJo

        // Determine the appropriate lift power.
        controller.setPID(PROPORTIONAL, INTEGRAL, DERIVATIVE);
        int currentPosition = leftMotor.getCurrentPosition();
        double pid = controller.calculate(currentPosition, targetPosition);
        double power = pid + FEEDFORWARD;

        // Set the lift's power.
        leftMotor.setPower(power);
        rightMotor.setPower(power);

        // Reset the lift if appropriate
        //////////////////////////////////////////////////////////////////////

        // If we finished lowering the lift...
        if (targetPosition == MINIMUM_POSITION && touch.isPressed()) {

            // Reset the lift.
            reset();

        }

        // Display lift telemetry.
        //////////////////////////////////////////////////////////////////////

        // Get the lift's position.
        int leftPosition = leftMotor.getCurrentPosition();
        int rightPosition = rightMotor.getCurrentPosition();

        // Get the lift's power.
        double leftPower = leftMotor.getPower();
        double rightPower = rightMotor.getPower();

        // Determine whether the lift is down.
        boolean down = touch.isPressed();

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Display lift telemetry.
        telemetry.addData("Lift", "====================");
        telemetry.addData("- Current Position", "%d/%d", leftPosition, rightPosition);
        telemetry.addData("- Down", down);
        telemetry.addData("- PID", pid);
        telemetry.addData("- Power", "%.2f/%.2f", leftPower, rightPower);
        telemetry.addData("- Target Position", targetPosition);

        // Add lift information to the telemetry.
        telemetry.addData("Lift", "Position = %d/%d, Power = %.2f/%.2f", leftPosition, rightPosition, leftPower, rightPower);

    }

    // Resets the lift.
    private void reset() {

        // Reset the lift motors.
        reset(leftMotor);
        reset(rightMotor);

    }

    // Resets a lift motor.
    private static void reset(DcMotorEx motor) {

        // Reset the motor.
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    }

    public double getPosition() {
        double leftPosition = leftMotor.getCurrentPosition();
        double rightPosition = rightMotor.getCurrentPosition();
        return (leftPosition + rightPosition) / 2;
    }

    public void overridePower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    // Determines whether the lift is in a position.
    public boolean isInPosition(int targetPosition) {

        int leftPosition = leftMotor.getCurrentPosition();
        int rightPosition = rightMotor.getCurrentPosition();
        int leftDifference = Math.abs(leftPosition - targetPosition);
        int rightDifference = Math.abs(rightPosition - targetPosition);
        if (leftDifference < THRESHOLD && rightDifference < THRESHOLD) {
            return true;
        } else {
            return false;
        }
    }

    // Raises the lift.
    public void raise() {

        // Raise the lift.
        targetPosition = Math.min(targetPosition + POSITION_INCREMENT, MAXIMUM_POSITION);

    }

    // Lowers the lift.
    public void lower() {

        // Lower the lift.
        targetPosition = Math.max(targetPosition - POSITION_INCREMENT, MINIMUM_POSITION);

    }

    // Waits for the user to lower the lift.
    public void waitForDown() {

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // While the lift is up...
        while (!opMode.isStopRequested() && !touch.isPressed()) {

            // Instruct the user to lower the lift.
            robotHardware.log("Please lower the lift...");

        }

        // Reset the lift.
        reset();

        // Notify the user that the lift is down.
        robotHardware.log("Lift is down");

    }

    // Moves the lift to the ground position.
    public void setGroundPosition() {

        // Move the lift to the minimum position.
        targetPosition = MINIMUM_POSITION;

    }

    // Moves the lift to the high basket position.
    public void setHighBasketPosition() {

        // Move the lift to the high basket position.
        targetPosition = HIGH_BASKET_POSITION;

    }

    public boolean targetingScoringPos() {
        return targetPosition > MINIMUM_POSITION;
    }
}