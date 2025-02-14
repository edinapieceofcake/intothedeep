package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// This represents a slide.
@Config
public class Slide2 {

    // Derivative coefficient
    public static double DERIVATIVE = 0.0001;

    // Feedforward coefficient
    public static double FEEDFORWARD = 0.1;

    public static double AMPS = -1;

    // Integral coefficient
    public static double INTEGRAL = 0;

    // Minimum position
    public static int MINIMUM_POSITION = -200;

    // Basket position
    public static int BASKET_POSITION = 1800;

    // Maximum submersible position
    public static int MAXIMUM_SUBMERSIBLE_POSITION = 1800;

    // Submersible position
    public static int SUBMERSIBLE_POSITION = 800;;

    // Chamber position
    public static int CHAMBER_POSITION = 100;

    // Position increment
    public static int POSITION_INCREMENT = 100;

    // Proportional coefficient
    public static double PROPORTIONAL = 0.002;

    // Busy threshold
    public static int BUSY_THRESHOLD = 150;

    // Rezeroing power
    public static double REZEROING_POWER = -0.3;

    // Auto position
    public static int AUTO_POSITION = 500;

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


        // Initialize the slide controller.
        controller = new PIDController(PROPORTIONAL, INTEGRAL, DERIVATIVE);

    }

    // Update this.
    public void update() {

        // Update the slide controller.
        //////////////////////////////////////////////////////////////////////

        // PIDF Loops & Arm Control | FTC | 16379 KookyBotz
        // https://www.youtube.com/watch?v=E6H6Nqe6qJo

        // Determine the appropriate slide power.
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

        // Determine whether the slide is busy.
        boolean isBusy = isBusy();

        // Display slide telemetry.
        telemetry.addData("Slide", "====================");
        telemetry.addData("- Busy", isBusy);
        telemetry.addData("- Current Position", "%d", extensionPosition);
        telemetry.addData("- PID", pid);
        telemetry.addData("- Power", "%.2f",  extensionMotorPower);
        telemetry.addData("- Target Position", targetPosition);

    }

    // Resets the slide.
    private void reset() {

        // Reset the extension motors.
        reset(extensionMotor);

    }

    // Resets a slide motor.
    private void reset(DcMotorEx motor) {

        // Reset the motor.
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        targetPosition = 0;

    }

    // Gets the slide's current position.
    public int getCurrentPosition() {

        // Return the slide's current position.
        return extensionMotor.getCurrentPosition();

    }

    // Gets the slide's target position.
    public int getTargetPosition() {

        // Return the slide's target position.
        return targetPosition;

    }

    // Determines whether the slide is busy.
    public boolean isBusy() {

        // Get the slide's current position.
        double currentPosition = getCurrentPosition();

        // Get the position difference.
        double difference = Math.abs(currentPosition - targetPosition);

        // Return indicating if the slide is busy.
        return difference >= BUSY_THRESHOLD;

    }

    // Raises the slide.
    public void raise() {

        // If the slide is fully raised...
        if (targetPosition + POSITION_INCREMENT > MAXIMUM_SUBMERSIBLE_POSITION) {

            // Notify the user.
            robotHardware.beep();

            // Exit the method.
            return;

        }

        // Raise the slide.
        targetPosition += POSITION_INCREMENT;

    }

    // Lowers the slide.
    public void lower() {

        // If the slide is fully lowered...
        if (targetPosition - POSITION_INCREMENT < MINIMUM_POSITION) {

            // Notify the user.
            robotHardware.beep();

            // Exit the method.
            return;

        }

        // Lower the slide.
        targetPosition -= POSITION_INCREMENT;

    }


    // Moves the slide to the minimum position.
    public void setMinimum() {

        // Move the slide to the minimum position.
        targetPosition = MINIMUM_POSITION;

    }

    // Moves the slide to the basket position.
    public void setBasket() {

        // Move the slide to the basket position.
        targetPosition = BASKET_POSITION;

    }

    // Moves the slide to the submersible position.
    public void setSubmersible() {

        // Move the slide to the submersible position.
        targetPosition = SUBMERSIBLE_POSITION;

    }

    // Moves the slide to the auto position.
    public void setAuto() {

        // Move the slide to the auto position.
        targetPosition = AUTO_POSITION;

    }

    // Moves the slide to the chamber position.
    public void setChamber() {

        // Move the slide to the chamber position.
        targetPosition = CHAMBER_POSITION;

    }

    // Sets the slide's target position.
    public void setTargetPosition(int targetPosition) {

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

    public void turnOff() {
        extensionMotor.setMotorDisable();
    }
}