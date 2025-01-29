package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// This represents an arm.
@Config
public class Arm {
    // Derivative coefficient
    public static double DERIVATIVE = 0.00005;

    // Feedforward coefficient
    public static double FEEDFORWARD = 0.1;

    // Basket position
    public static int BASKET_POSITION = 2200;

    // Integral coefficient
    public static double INTEGRAL = 0;

    // Initial degrees below horizontal (determined experimentally)
    public static double INITIAL_DEGREES_BELOW_HORIZONTAL = 30;

    // Maximum position
    public static int MAXIMUM_POSITION = 5000;

    // Minimum position
    public static int MINIMUM_POSITION = 0;

    // Ground position
    public static int GROUND_POSITION = MINIMUM_POSITION;

    // Proportional coefficient
    public static double PROPORTIONAL = 0.0008;

    // Position increment (ticks)
    public static int POSITION_INCREMENT = 50;

    // Rezeroing power
    public static double REZEROING_POWER = -0.3;

    // Submersible enter position
    public static int SUBMERSIBLE_ENTER_POSITION = 4600;

    // Submersible hover position
    public static int SUBMERSIBLE_HOVER_POSITION = 4300;

    // Submersible grab position
    public static int SUBMERSIBLE_GRAB_POSITION = 5000;

    // Submersible position threshold
    public static int SUBMERSIBLE_POSITION_THRESHOLD = 1000;

    // Ticks per degree (determined experimentally)
    public static double TICKS_PER_DEGREE = 23.3;

    // Ground to tall wall position
    public static int GROUND_TO_TALL_WALL_POSITION = 500;

    // Submersible to tall wall position
    public static int SUBMERSIBLE_TO_TALL_WALL_POSITION = 400;

    // Ground to short wall position
    public static int GROUND_TO_SHORT_WALL_POSITION = 450;

    // Submersible to short wall position
    public static int SUBMERSIBLE_TO_SHORT_WALL_POSITION = 300;

    // Chamber position
    public static int CHAMBER_POSITION = 5000;

    // Ascent position
    public static int ASCENT_POSITION = 700;

    // Busy threshold
    public static int BUSY_THRESHOLD = 600;

    // Controller
    private PIDController controller;

    // Motor
    private final DcMotorEx motor;

    // Rezeroing
    private boolean rezeroing;

    // Robot hardware
    private final RobotHardware robotHardware;

    // Target position
    private int targetPosition;

    // Touch sensor
    private final TouchSensor touchFront;
    private final TouchSensor touchBack;

    // Previous front down value
    private boolean previousFrontDown;

    // Initializes this.
    public Arm(RobotHardware robotHardware) {

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Get the motor.
        motor = hardwareMap.get(DcMotorEx.class, "arm_motor");

        // Reverse the motor.
        motor.setDirection(DcMotorEx.Direction.FORWARD);

        // Reset the motor.
        reset();

        // Get the touch sensor.
        touchFront = hardwareMap.get(TouchSensor.class, "arm_touch_front");
        touchBack = hardwareMap.get(TouchSensor.class, "arm_touch_back");

        // Initialize the arm controller.
        controller = new PIDController(PROPORTIONAL, INTEGRAL, DERIVATIVE);

    }

    // Updates this.
    public void update() {

        // Update the arm controller.
        //////////////////////////////////////////////////////////////////////

        // PIDF Loops & Arm Control | FTC | 16379 KookyBotz
        // https://www.youtube.com/watch?v=E6H6Nqe6qJo

        // Determine the appropriate arm power.
        controller.setPID(PROPORTIONAL, INTEGRAL, DERIVATIVE);
        int currentPosition = motor.getCurrentPosition();
        double currentDegrees = getDegrees(currentPosition);
        double currentRadians = Math.toRadians(currentDegrees);
        double pid = controller.calculate(currentPosition, targetPosition);
        double targetDegrees = getDegrees(targetPosition);
        double feedForward = Math.cos(currentRadians) * FEEDFORWARD;
        double power = pid + feedForward;

        // Set the motor power.
        //////////////////////////////////////////////////////////////////////

        // If we are rezeroing...
        if (rezeroing) {

            // Use the rezeroing power.
            motor.setPower(REZEROING_POWER);

        }

        // Otherwise (if we are not rezeroing)...
        else {

            // Use the controller's power.
            motor.setPower(power);

        }

        // Reset the arm if appropriate
        //////////////////////////////////////////////////////////////////////

        // Determine whether the arm is down.
        boolean currentFrontDown = touchFront.isPressed();

        // If the arm is down...
        if (currentFrontDown && !previousFrontDown) {

            // Reset the arm motor.
            reset();

        }

        previousFrontDown = currentFrontDown;

        boolean backDown = touchBack.isPressed();

        // Display arm telemetry.
        //////////////////////////////////////////////////////////////////////

        // Determine whether the arm is busy.
        boolean isBusy = isBusy();

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Display arm telemetry.
        telemetry.addData("Arm", "====================");
        telemetry.addData("- Busy", isBusy);
        telemetry.addData("- Current Degrees", currentDegrees);
        telemetry.addData("- Current Position", currentPosition);
        telemetry.addData("- Front Down", currentFrontDown);
        telemetry.addData("- Back Down", backDown);
        telemetry.addData("- Feedforward", feedForward);
        telemetry.addData("- PID", pid);
        telemetry.addData("- Power", power);
        telemetry.addData("- Target Degrees", targetDegrees);
        telemetry.addData("- Target Position", targetPosition);

    }

    // Converts the arm motor's ticks to degrees.
    private double getDegrees(double ticks) {

        // Convert the arm motor's ticks to degrees.
        double degrees = ticks / TICKS_PER_DEGREE - INITIAL_DEGREES_BELOW_HORIZONTAL;

        // Return the result.
        return degrees;

    }

    // Waits for the user to lower the arm.
    public void waitForDown() {

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // While the arm is up...
        while (!opMode.isStopRequested() && !touchFront.isPressed()) {

            // Instruct the user to lower the arm.
            robotHardware.log("Please lower the arm...");

        }

        // Reset the arm.
        reset();

        // Notify the user that the arm is down.
        robotHardware.log("Arm is down");

    }

    // Resets the arm motor.
    private void reset() {

        // Reset the arm motor.
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    }

    // Decrements the arm position.
    public void decrementPosition() {

        // If the arm is fully lowered...
        if (targetPosition - POSITION_INCREMENT < MINIMUM_POSITION) {

            // Notify the user.
            robotHardware.beep();

            // Exit the method.
            return;

        }

        // Decrement the arm position.
        targetPosition -= POSITION_INCREMENT;

    }

    // Increments the arm position.
    public void incrementPosition() {

        // If the arm is fully raised...
        if (targetPosition + POSITION_INCREMENT > MAXIMUM_POSITION) {

            // Notify the user.
            robotHardware.beep();

            // Exit the method.
            return;

        }

        // Increment the arm position.
        targetPosition += POSITION_INCREMENT;

    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public void overridePower(double power) {
        motor.setPower(power);
    }

    // Sets the arm's position.
    public void setPosition(int position) {

        // Set the target position.
        targetPosition = position;

    }

    // Moves the arm to the ascent position.
    public void setAscentPosition() {

        // Move the arm to the ascent position.
        targetPosition = ASCENT_POSITION;

    }

    // Determines whether the arm is busy.
    public boolean isBusy() {

        // Get the arm's current position.
        int currentPosition = motor.getCurrentPosition();

        // Get the position difference.
        int difference = Math.abs(currentPosition - targetPosition);

        // Return indicating if the arm is busy.
        return difference >= BUSY_THRESHOLD;

    }

    // Starts rezeroing.
    public void startRezeroing() {

        // Start rezeroing.
        rezeroing = true;

    }

    // Stops rezeroing.
    public void stopRezeroing() {

        // If we are not rezeroing...
        if (!rezeroing) {

            // Exit the method.
            return;

        }

        // Reset this.
        reset();

        // Stop rezeroing.
        rezeroing = false;
    }

}