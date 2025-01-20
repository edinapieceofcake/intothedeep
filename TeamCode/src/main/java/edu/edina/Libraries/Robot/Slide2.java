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
public class Slide2 {

    // Derivative coefficient
    public static double DERIVATIVE = 0.0001;

    // Feedforward coefficient
    public static double FEEDFORWARD = 0.1;

    // Integral coefficient
    public static double INTEGRAL = 0;

    // Minimum position
    public static int MINIMUM_POSITION = -200;

    // High basket position
    public static int HIGH_BASKET_POSITION = 1900;

    // Chamber position
    public static int CHAMBER_POSITION = 100;

    // Nearly up position
    public static int NEARLY_UP_POSITION = HIGH_BASKET_POSITION - 200;

    // Position increment
    public static int POSITION_INCREMENT = 50;

    // Proportional coefficient
    public static double PROPORTIONAL = 0.002;

    // Busy threshold
    public static int BUSY_THRESHOLD = 150;
    public static int INITIALIZE_POSITION = 500;

    // Rezeroing power
    public static double REZEROING_POWER = -0.3;

    // Controller
    private PIDController controller;

    // Rezeroing
    private boolean rezeroing;

    // Right motor
    private final DcMotorEx extensionMotor;

    // Robot hardware
    private RobotHardware robotHardware;

    // Target position
    private int targetPosition;



    // Initializes this.
    public Slide2(RobotHardware robotHardware) {

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;


        // Get the right motor.
        extensionMotor = hardwareMap.get(DcMotorEx.class, "extension_motor");

        // Set the right motor's direction.
        extensionMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Reset the motors.
        reset();


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
        int currentPosition = extensionMotor.getCurrentPosition();
        double pid = controller.calculate(currentPosition, targetPosition);
        double power = pid + FEEDFORWARD;

        // Set the motor power.
        //////////////////////////////////////////////////////////////////////

        // If we are rezeroing...
        if (rezeroing) {

            // Use the rezeroing power.
            extensionMotor.setPower(REZEROING_POWER);
        }

        // Otherwise (if we are not rezeroing)...
        else {

            // Use the controller's power.
            extensionMotor.setPower(power);

        }
        int extensionPosition = extensionMotor.getCurrentPosition();

        // Get the extension's power.
        double extensionMotorPower = extensionMotor.getPower();


        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Determine whether the lift is busy.
        boolean isBusy = isBusy();

        // Display lift telemetry.
        telemetry.addData("Extension", "====================");
        telemetry.addData("- Busy", isBusy);
        telemetry.addData("- Current Position", "%d", extensionPosition);
        telemetry.addData("- PID", pid);
        telemetry.addData("- Power", "%.2f",  extensionMotorPower);
        telemetry.addData("- Target Position", targetPosition);

        // Add lift information to the telemetry.
        telemetry.addData("Extension", "Position = %d, Power = %.2f", extensionPosition, extensionMotorPower);

    }

    // Resets the lift.
    private void reset() {

        // Reset the extension motors.
        reset(extensionMotor);

    }

    // Resets a lift motor.
    private void reset(DcMotorEx motor) {

        // Reset the motor.
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        targetPosition = 0;

    }

    public double getPosition() {
        double extensionPosition = extensionMotor.getCurrentPosition();
        return extensionPosition;
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
        if (targetPosition + POSITION_INCREMENT > HIGH_BASKET_POSITION) {

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
        if (targetPosition - POSITION_INCREMENT < MINIMUM_POSITION) {

            // Notify the user.
            robotHardware.beep();

            // Exit the method.
            return;

        }

        // Lower the lift.
        targetPosition -= POSITION_INCREMENT;

    }


    // Moves the lift to the ground position.
    public void setMinimumExtension() {

        // Move the lift to the minimum position.
        targetPosition = MINIMUM_POSITION;

    }

    // Moves the lift to the high basket position.
    public void setMaximumExtension() {

        // Move the lift to the high basket position.
        targetPosition = HIGH_BASKET_POSITION;

    }
    // Moves the lift to the high basket position.
    public void setInitializeExtension() {

        // Move the lift to the initialize position.
        targetPosition = INITIALIZE_POSITION;

    }

    // Moves the lift to the high basket position.
    public void setChamber() {

        // Move the lift to the chamber position.
        targetPosition = CHAMBER_POSITION;

    }

    // Sets the lift's position.
    public void setPosition(int targetPosition) {

        // Set the target position.
        this.targetPosition = targetPosition;

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