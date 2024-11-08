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

    // Ground position
    public static int GROUND_POSITION = -400;

    // High basket position
    public static int HIGH_BASKET_POSITION = 2800;

    // Integral coefficient
    public static double INTEGRAL = 0;

    // Initial degrees below horizontal (determined experimentally)
    public static double INITIAL_DEGREES_BELOW_HORIZONTAL = 26;

    // Low basket position
    public static int LOW_BASKET_POSITION = 2700;

    // Nearly down position threshold
    public static int NEARLY_DOWN_POSITION = 100;

    // Proportional coefficient
    public static double PROPORTIONAL = 0.0005;

    // Position increment (ticks)
    public static int POSITION_INCREMENT = 25;

    // Submersible position
    public static int SUBMERSIBLE_POSITION = 4900;

    // Ticks per degree (determined experimentally)
    public static double TICKS_PER_DEGREE = 23.3;

    // Wall position
    public static int WALL_POSITION = 400;

    // Positions
    public static int[] POSITIONS = new int[] {
            GROUND_POSITION,
            WALL_POSITION,
            HIGH_BASKET_POSITION,
            LOW_BASKET_POSITION,
            SUBMERSIBLE_POSITION
    };

    // Controller
    private PIDController controller;

    // Motor
    private final DcMotorEx motor;

    // Robot hardware
    private final RobotHardware robotHardware;

    // Target position
    private int targetPosition;

    // Touch sensor
    private final TouchSensor touch;

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
        motor.setDirection(DcMotorEx.Direction.REVERSE);

        // Reset the motor.
        reset();

        // Get the touch sensor.
        touch = hardwareMap.get(TouchSensor.class, "arm_touch");

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

        // Set the arm's power.
        motor.setPower(power);

        // Reset the arm if appropriate
        //////////////////////////////////////////////////////////////////////

        // Determine whether the arm is down.
        boolean down = touch.isPressed();

        // If the arm is down...
        if(down) {

            // Reset the arm motor.
            reset();

        }

        // Display arm telemetry.
        //////////////////////////////////////////////////////////////////////

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Display arm telemetry.
        telemetry.addData("Arm", "====================");
        telemetry.addData("- Current Degrees", currentDegrees);
        telemetry.addData("- Current Position", currentPosition);
        telemetry.addData("- Down", down);
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
        while (!opMode.isStopRequested() && !touch.isPressed()) {

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

    // Goes to the previous arm position.
    public void previousPosition() {

        // Count the arm positions.
        int count = POSITIONS.length;

        // For each arm position...
        for(int index = count - 1; index >= 0; index--) {

            // Get the current arm position.
            int currentPosition = POSITIONS[index];

            // If the current arm position is less than the target...
            if(currentPosition < targetPosition) {

                // Use the current arm position as the target.
                targetPosition = currentPosition;

                // Exit the method.
                return;

            }

        }

    }

    // Goes to the next arm position.
    public void nextPosition() {

        // Count the arm positions.
        int count = POSITIONS.length;

        // For each arm position...
        for(int index = 0; index < count; index++) {

            // Get the current arm position.
            int currentPosition = POSITIONS[index];

            // If the current arm position is greater than the target...
            if(currentPosition > targetPosition) {

                // Use the current arm position as the target.
                targetPosition = currentPosition;

                // Exit the method.
                return;

            }

        }

    }

    // Decrements the arm position.
    public void decrementPosition() {

        // Decrement the arm position.
        targetPosition -= POSITION_INCREMENT;

    }

    // Increments the arm position.
    public void incrementPosition() {

        // Increment the arm position.
        targetPosition += POSITION_INCREMENT;

    }

    // Sets the arm's position.
    public void setPosition(int position) {

        // Set the target position.
        targetPosition = position;

    }

    // Moves the arm to the ground position.
    public void setGroundPosition() {

        // Move the arm to the ground position.
        targetPosition = GROUND_POSITION;

    }

    // Moves the arm to the low basket position.
    public void setLowBasketPosition() {

        // Move the arm to the low basket position.
        targetPosition = LOW_BASKET_POSITION;

    }

    // Moves the arm to the high basket position.
    public void setHighBasketPosition() {

        // Move the arm to the high basket position.
        targetPosition = HIGH_BASKET_POSITION;

    }

    // Determines whether the arm is nearly down.
    public boolean isNearlyDown() {

        // Determine whether the arm is nearly down.
        boolean isNearlyDown = targetPosition <= NEARLY_DOWN_POSITION;

        // Return the result.
        return isNearlyDown;

    }

}