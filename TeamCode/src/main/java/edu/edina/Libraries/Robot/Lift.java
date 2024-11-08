package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// This represents a lift.
@Config
public class Lift {

    // Down position
    public static int DOWN_POSITION = 0;

    // Lower power
    public static double LOWER_POWER = -0.6;

    // Raise power
    public static double RAISE_POWER = 1;

    // Threshold
    public static int THRESHOLD = 50;

    // Left motor
    private final DcMotorEx leftMotor;

    // Op mode
    private LinearOpMode opMode;

    // Right motor
    private final DcMotorEx rightMotor;

    // Robot hardware
    private RobotHardware robotHardware;

    // Target position
    private int targetPosition;

    // Touch sensor
    private final TouchSensor touch;

    // Initializes this.
    public Lift(LinearOpMode opMode, RobotHardware robotHardware) {

        // Remember the op mode.
        this.opMode = opMode;

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Get the left motor.
        leftMotor = hardwareMap.get(DcMotorEx.class, "left_lift_motor");

        // Configure the left motor.
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Get the right motor.
        rightMotor = hardwareMap.get(DcMotorEx.class, "right_lift_motor");

        // Configure the right motor.
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Get the touch sensor.
        touch = hardwareMap.get(TouchSensor.class, "lift_touch");

    }

    // Update this.
    public void update() {

        // Get the lift's position.
        int leftPosition = leftMotor.getCurrentPosition();
        int rightPosition = rightMotor.getCurrentPosition();

        // If we finished lowering the lift...
        if(targetPosition == DOWN_POSITION && touch.isPressed()) {

            // Reset the lift.
            reset();

        }

        // Get the lift's power.
        double leftPower = leftMotor.getPower();
        double rightPower = rightMotor.getPower();

        // Determine whether the lift is down.
        boolean down = touch.isPressed();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Display lift telemetry.
        telemetry.addData("Lift", "====================");
        telemetry.addData("- Down", down);
        telemetry.addData("- Position", "%d/%d", leftPosition, rightPosition);
        telemetry.addData("- Power", "%.2f/%.2f", leftPower, rightPower);

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
    private static void reset(DcMotor liftMotor) {

        // Reset the lift motor.
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    // Sets the lift's power.
    public void setPower(double power) {
        setPower(leftMotor, power);
        setPower(rightMotor, power);
    }

    // Sets a motor's power.
    private static void setPower(DcMotorEx motor, double power) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(power);
    }

    // Sets the lift's position.
    public void setPosition(int targetPosition) {

        // Remember the target position.
        this.targetPosition = targetPosition;

        // Get the lift's current position.
        int currentPosition = leftMotor.getCurrentPosition();

        // Get the appropriate power.
        double power = targetPosition > currentPosition ? RAISE_POWER : LOWER_POWER;

        // Lower the lift.
        setPosition(leftMotor, targetPosition, power);
        setPosition(rightMotor, targetPosition, power);

    }

    // Sets the lift position.
    private void setPosition(DcMotor liftMotor, int position, double power) {

        // Set the lift position.
        liftMotor.setTargetPosition(position);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(power);

    }

    // Determines whether the lift is in a position.
    public boolean isInPosition(int targetPosition) {

        int leftPosition = leftMotor.getCurrentPosition();
        int rightPosition = rightMotor.getCurrentPosition();
        int leftDifference = Math.abs(leftPosition - targetPosition);
        int rightDifference = Math.abs(rightPosition - targetPosition);
        if (leftDifference < THRESHOLD && rightDifference < THRESHOLD) {
            return true;
        }
        else {
            return false;
        }
    }

    // Gets the lift's position.
    public int getLiftPosition() {
        return leftMotor.getCurrentPosition();
    }

    // Raises the lift.
    public void raise() {

        // Raise the lift.
        setPower(RAISE_POWER);

    }

    // Lowers the lift.
    public void lower() {

        // Lower the lift.
        setPower(LOWER_POWER);

    }

    // Stops the lift.
    public void stop() {

        // Stop the lift.
        setPower(0);

    }

    // Waits for the user to lower the lift.
    public void waitForDown() {

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

}