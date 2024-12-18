package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Slide {

    // Extension error threshold
    public static double EXTENSION_ERROR_THRESHOLD = 0.5;

    // Extension increment.
    public static double EXTENSION_INCREMENT = 1;

    // Inches per volt
    public static double INCHES_PER_VOLT = 6.375 / 4.4;

    // Low basket extension
    public static double LOW_BASKET_EXTENSION = 5;

    // Low voltage divisor
    public static double LOW_VOLTAGE_DIVISOR = 4;

    // High basket extension
    public static double HIGH_BASKET_EXTENSION = 9;

    // Maximum extension
    public static double MAXIMUM_EXTENSION = 10;

    // Minimum extension
    public static double MINIMUM_EXTENSION = 0;

    // Maximum power
    public static double MAXIMUM_POWER = 1;

    // Maximum voltage difference
    public static double MAXIMUM_VOLTAGE_DIFFERENCE = 1;

    // Proportional coefficient
    public static double PROPORTIONAL = 0.5;

    // Uninitialized value
    private static final double UNINITIALIZED = -1;

    // Valid voltage threshold
    private static final double VALID_VOLTAGE_THRESHOLD = 0.1;

    // Zero power threshold
    private static final double ZERO_POWER_THRESHOLD = 0.0001;

    // Rezeroing power
    public static double REZEROING_POWER = 0.2;

    // Encoder
    private final AnalogInput encoder;

    // Target extension
    private double targetExtension;

    // Last voltage
    private double lastVoltage = UNINITIALIZED;

    // Offset voltage
    private double offsetVoltage = 0;

    // Rezeroing
    private boolean rezeroing;

    // Robot hardware
    private final RobotHardware robotHardware;

    // Servo
    private final CRServo servo;

    // Initializes this.
    public Slide(RobotHardware robotHardware) {

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Get the servo.
        servo = hardwareMap.get(CRServo.class, "slide_servo");

        // Get the encoder.
        encoder = hardwareMap.get(AnalogInput.class, "slide_encoder");

        // Set the servo's power so it begins reporting voltages.
        servo.setPower(0);

    }

    // return true if a reading was made
    public boolean updateVoltage() {

        // Get the current voltage.
        double currentVoltage = encoder.getVoltage();

        // Get the maximum voltage.
        double maximumVoltage = encoder.getMaxVoltage();

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // If the current voltage is invalid...
        if (currentVoltage < -VALID_VOLTAGE_THRESHOLD || currentVoltage > maximumVoltage + VALID_VOLTAGE_THRESHOLD) {

            // Display an error message.
            telemetry.addData("Slide Error", "Voltage " + currentVoltage + " is invalid.  Max voltage is " + maximumVoltage + ".");

            // Exit the method.
            return false;

        }

        // If the last voltage is uninitialized...
        if (lastVoltage == UNINITIALIZED) {

            // If we have a current voltage...
            if (currentVoltage > ZERO_POWER_THRESHOLD) {

                // Initialize the last voltage.
                lastVoltage = currentVoltage;

            }

            // Otherwise (if we do not have a current voltage)...
            else {

                // Display an error message.
                telemetry.addData("Slide Error", "Voltage " + currentVoltage + " is less than epsilon.");

            }

            // Exit the method.
            return false;

        }

        // Get a low voltage threshold.
        double lowVoltage = maximumVoltage / LOW_VOLTAGE_DIVISOR;

        // Get a high voltage threshold.
        double highVoltage = maximumVoltage - lowVoltage;

        // Update the offset voltage.
        double newOffsetVoltage = offsetVoltage + currentVoltage - lastVoltage;

        // If we wrapped around from high to low voltage...
        if (currentVoltage < lowVoltage && lastVoltage > highVoltage) {

            // Update the offset voltage.
            newOffsetVoltage += maximumVoltage;

        }

        // Otherwise, if we wrapped around from low to high voltage...
        else if (currentVoltage > highVoltage && lastVoltage < lowVoltage) {

            // Update the offset voltage.
            newOffsetVoltage -= maximumVoltage;

        }

        // Get the offset voltage difference.
        double offsetVoltageDifference = Math.abs(newOffsetVoltage - offsetVoltage);

        // If the offset voltage difference is too high...
        if (offsetVoltageDifference > MAXIMUM_VOLTAGE_DIFFERENCE) {

            // Display an error message.
            telemetry.addData("Slide Error", "Offset voltage difference " + offsetVoltageDifference + " is too large.");

            // Exit the method.
            return false;

        }

        // Update the offset voltage.
        offsetVoltage = newOffsetVoltage;

        // Update the last voltage.
        lastVoltage = currentVoltage;

        return true;
    }

    // Updates this.
    public void update() {

        // If updating the voltage failed...
        if (!updateVoltage()) {

            // Exit the method.
            return;

        }

        // Get the current extension.
        double currentExtension = getCurrentExtension();

        // Get the extension error.
        double extensionError = currentExtension - targetExtension;

        // Get a power.
        double inputPower = clamp(extensionError * PROPORTIONAL, -MAXIMUM_POWER, MAXIMUM_POWER);

        // If we are rezeroing...
        if(rezeroing) {

            // Use the rezeroing power.
            servo.setPower(REZEROING_POWER);

        }

        else {

            // Use the input power.
            servo.setPower(inputPower);

        }

        // Get the servo's power.
        double outputPower = servo.getPower();

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Determine whether the slide is busy.
        boolean isBusy = isBusy();

        // Display extension telemetry.
        telemetry.addData("Slide", "====================");
        telemetry.addData("- Busy", isBusy);
        telemetry.addData("- Current Extension", currentExtension);
        telemetry.addData("- Current Voltage", lastVoltage);
        telemetry.addData("- Extension Error", extensionError);
        telemetry.addData("- Extension Target", targetExtension);
        telemetry.addData("- Input Power", inputPower);
        telemetry.addData("- Offset Voltage", offsetVoltage);
        telemetry.addData("- Output Power", outputPower);

    }

    public void overridePower(double power) {
        servo.setPower(power);
    }

    // Sets the extension's position.
    public void setPosition(double position) {

        // Set the extension's position.
        this.targetExtension = position;

    }

    // Gets the slide's position.
    public double getPosition() {

        // Return the slide's position.
        return INCHES_PER_VOLT * offsetVoltage;

    }

    // Sets the minimum extension.
    public void setMinimumExtension() {

        // Set the minimum extension.
        targetExtension = MINIMUM_EXTENSION;

    }

    // Sets the low basket extension.
    public void setLowBasketExtension() {

        // Set the low basket extension.
        targetExtension = LOW_BASKET_EXTENSION;

    }

    // Sets the high basket extension.
    public void setHighBasketExtension() {

        // Set the high basket extension.
        targetExtension = HIGH_BASKET_EXTENSION;

    }

    // Extends this.
    public void extend() {

        // If the slide is fully extended...
        if(targetExtension + EXTENSION_INCREMENT > MAXIMUM_EXTENSION) {

            // Notify the user.
            robotHardware.beep();

            // Exit the method.
            return;

        }

        // Extend this.
        targetExtension += EXTENSION_INCREMENT;

    }

    // Retracts this.
    public void retract() {

        // If the slide is fully retracted...
        if(targetExtension - EXTENSION_INCREMENT < MINIMUM_EXTENSION) {

            // Notify the user.
            robotHardware.beep();

            // Exit the method.
            return;

        }

        // Retract this.
        targetExtension -= EXTENSION_INCREMENT;

    }

    // Determines whether the slide is busy.
    public boolean isBusy() {

        // Get the slide's current extension.
        double currentExtension = getCurrentExtension();

        // Get the extension difference.
        double difference = Math.abs(currentExtension - targetExtension);

        // Return indicating if the slide is busy.
        return difference >= EXTENSION_ERROR_THRESHOLD;

    }

    // Get the current exetension.
    public double getCurrentExtension() {

        // Return the current extension.
        return INCHES_PER_VOLT * offsetVoltage;

    }

    // Clamps a value.
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    // Determines whether this is fully extended.
    public boolean isFullyExtended() {
        return targetExtension == MAXIMUM_EXTENSION;
    }

    // Determines whether this is fully retracted.
    public boolean isFullyRetracted() {
        return targetExtension == MINIMUM_EXTENSION;
    }

    // Starts rezeroing.
    public void startRezeroing() {

        // Start rezeroing.
        rezeroing = true;

    }

    // Stops rezeroing.
    public void stopRezeroing() {

        // If we are not rezeroing...
        if(!rezeroing) {

            // Exit the method.
            return;

        }

        // Reset this.
        offsetVoltage = 0;

        // Stop rezeroing.
        rezeroing = false;

    }

}