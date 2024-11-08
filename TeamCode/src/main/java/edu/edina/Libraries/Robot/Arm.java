package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// This represents an arm.
@Config
public class Arm {

    // Derivative coefficient
    public static double D = 0.00005;

    // Feed forward coefficient
    public static double F = 0.1;

    // Integral coefficient
    public static double I = 0;

    // Initial degrees below horizontal (determined experimentally)
    public static double INITIAL_DEGREES_BELOW_HORIZONTAL = 26;

    // Proportional coefficient
    public static double P = 0.0005;

    // Position increment (ticks)
    public static int POSITION_INCREMENT = 25;

    // Positions
    public static int[] POSITIONS = new int[] {-400, 400, 3000, 4900};

    // Ticks per degree (determined experimentally)
    public static double TICKS_PER_DEGREE = 23.3;

    // Controller
    private PIDController controller;

    // Motor
    private final DcMotorEx motor;

    // Robot hardware
    private final RobotHardware robotHardware;

    // Target position
    private int targetPosition = 0;

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
        motor.setDirection(DcMotor.Direction.REVERSE);

        // Configure the motor.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Get the touch sensor.
        touch = hardwareMap.get(TouchSensor.class, "arm_touch");

        // Initialize the arm controller.
        controller = new PIDController(P, I, D);

    }

    // Updates this.
    public void update() {

        // PIDF Loops & Arm Control | FTC | 16379 KookyBotz
        // https://www.youtube.com/watch?v=E6H6Nqe6qJo

        // Determine the appropriate arm power.
        controller.setPID(P, I, D);
        int actualPosition = motor.getCurrentPosition();
        double actualDegrees = getDegrees(actualPosition);
        double actualRadians = Math.toRadians(actualDegrees);
        double pid = controller.calculate(actualPosition, targetPosition);
        double targetDegrees = getDegrees(targetPosition);
        double feedForward = Math.cos(actualRadians) * F;
        double power = pid + feedForward;

        // Set the arm's power.
        motor.setPower(power);

        // Determine whether the arm is down.
        boolean down = touch.isPressed();

        // If the arm is down...
        if(down) {

            // Reset the arm position to zero.
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Display arm telemetry.
        telemetry.addData("Arm", "====================");
        telemetry.addData("- Down", down);
        telemetry.addData("- PID", pid);
        telemetry.addData("- Feed Forward", feedForward);
        telemetry.addData("- Power", power);
        telemetry.addData("- Actual Position", actualPosition);
        telemetry.addData("- Actual Degrees", actualDegrees);
        telemetry.addData("- Target Position", targetPosition);
        telemetry.addData("- Target Degrees", targetDegrees);

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
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

    // Move the arm to its first position.
    public void setFirstPosition() {

        // Move the arm to its first position.
        targetPosition = POSITIONS[0];

    }

}