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

    // Integral coefficient
    public static double INTEGRAL = 0;

    // Maximum position
    public static int MAXIMUM_POSITION = 1800;

    // Minimum position
    public static int MINIMUM_POSITION = 0;

    // Up position threshold
    public static int RAISED_POSITION_THRESHOLD = 400;

    // Nearly up position
    public static int NEARLY_UP_POSITION = 1500;

    // Position increment
    public static int POSITION_INCREMENT = 50;

    // Proportional coefficient
    public static double PROPORTIONAL = 0.002;

    // Busy threshold
    public static int BUSY_THRESHOLD = 150;

    // High basket position
    public static int HIGH_BASKET_POSITION = MAXIMUM_POSITION;

    // Descend position
    public static int DESCEND_POSITION = 1500;

    // Ascend position
    public static int ASCEND_POSITION = -400;

    private static double INCHES_PER_POS = 14.835 / 1679.0;

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

        // Determine whether the lift is busy.
        boolean isBusy = isBusy();

        // Display lift telemetry.
        telemetry.addData("Lift", "====================");
        telemetry.addData("- Busy", isBusy);
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

    public double getPositionInInches() {
        return getPosition() * INCHES_PER_POS;
    }

    public void overridePower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    // Determines whether the lift is busy.
    public boolean isBusy() {

        // Get the lift's current position.
        double currentPosition = getPosition();

        // If the lift is up as requested...
        if (currentPosition >= NEARLY_UP_POSITION && targetPosition >= NEARLY_UP_POSITION) {

            // Return indicating that the lift is not busy.
            return false;

        }

        // Get the position difference.
        double difference = Math.abs(currentPosition - targetPosition);

        // Return indicating if the lift is busy.
        return difference >= BUSY_THRESHOLD;

    }

    // Raises the lift.
    public void raise() {

        // If the lift is fully raised...
        if(targetPosition + POSITION_INCREMENT > MAXIMUM_POSITION) {

            // Notify the user.
            robotHardware.beep();

            // Exit the method.
            return;

        }

        // Raise the lift.
        targetPosition += POSITION_INCREMENT;

    }

    // Lowers the lift.
    public void lower() {

        // If the lift is fully lowered...
        if(targetPosition - POSITION_INCREMENT < MINIMUM_POSITION) {

            // Notify the user.
            robotHardware.beep();

            // Exit the method.
            return;

        }

        // Lower the lift.
        targetPosition -= POSITION_INCREMENT;

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

    // Determines whether the lift is raised.
    public boolean isRaised() {

        // Get the lift's current position.
        double currentPosition = getPosition();

        // Determine whether the lift is raised.
        boolean isRaised = currentPosition > RAISED_POSITION_THRESHOLD;

        // Return the result.
        return isRaised;

    }

    public boolean targetingScoringPos() {
        return targetPosition > MINIMUM_POSITION;
    }

    // Determines whether the lift is in the high basket position.
    public boolean isInHighBasketPosition() {

        // Determine whether the lift is in the high basket position.
        boolean isInHighBasketPosition = targetPosition == HIGH_BASKET_POSITION;

        // Return the result.
        return isInHighBasketPosition;

    }

    // Determines whether the lift is in the ground position.
    public boolean isInGroundPosition() {

        // Determine whether the lift is in the ground position.
        boolean isInGroundPosition = targetPosition == MINIMUM_POSITION;

        // Return the result.
        return isInGroundPosition;

    }

    // Sets the lift's position.
    public void setPosition(int targetPosition) {

        // Set the target position.
        this.targetPosition = targetPosition;

    }

}