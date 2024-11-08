package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ArmExtension {

    // Epsilon
    private static final double EPSILON = 0.0001;

    // Extension error threshold
    public static double EXTENSION_ERROR_THRESHOLD = 0.5;

    // Inches per volt
    public static double INCHES_PER_VOLT = 6.375 / 4.4;

    // Power
    public static double POWER = 1;

    // Uninitialized value
    private static final double UNINITIALIZED = -1;

    // Encoder
    private final AnalogInput encoder;

    // Extension target
    private double extensionTarget;

    // Last voltage
    private double lastVoltage = UNINITIALIZED;

    // Offset voltage
    private double offsetVoltage = 0;

    // Robot hardware
    private final RobotHardware robotHardware;

    // Servo
    private final CRServo servo;

    // Initializes this.
    public ArmExtension(RobotHardware robotHardware) {

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

    // Updates this.
    public void update() {

        // Get the current voltage.
        double currentVoltage = encoder.getVoltage();

        // Get the maximum voltage.
        double maximumVoltage = encoder.getMaxVoltage();

        if(currentVoltage < 0 || currentVoltage > maximumVoltage) {
            return;
        }

        // If the last voltage is uninitialized...
        if(lastVoltage == UNINITIALIZED) {

            // If we have a current voltage...
            if(currentVoltage > EPSILON) {

                // Initialize the last voltage.
                lastVoltage = currentVoltage;

            }

            // Exit the method.
            return;

        }

        // Get a low voltage threshold.
        double lowVoltage = maximumVoltage / 4;

        // Get a high voltage threshold.
        double highVoltage = maximumVoltage - lowVoltage;

        // Update the offset voltage.
        offsetVoltage += currentVoltage - lastVoltage;

        // If we wrapped around from high to low voltage...
        if (currentVoltage < lowVoltage && lastVoltage > highVoltage) {

            // Update the offset voltage.
            offsetVoltage += maximumVoltage;

        }

        // Otherwise, if we wrapped around from low to high voltage...
        else if (currentVoltage > highVoltage && lastVoltage < lowVoltage) {

            // Update the offset voltage.
            offsetVoltage -= maximumVoltage;

        }

        // Update the last voltage.
        lastVoltage = currentVoltage;

        // Get the current extension.
        double currentExtension = INCHES_PER_VOLT * offsetVoltage;

        // Get the extension error.
        double extensionError = currentExtension - extensionTarget;

        // Get a power.
        double inputPower;
        if(Math.abs(extensionError) < EXTENSION_ERROR_THRESHOLD) {
            inputPower = 0;
        }
        else {
            inputPower = extensionError > 0 ? POWER : -POWER;
        }

        // Set the power.
        servo.setPower(inputPower);

        // Get the servo's power.
        double outputPower = servo.getPower();

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Display extension telemetry.
        telemetry.addData("Extension", "====================");
        telemetry.addData("- Current Extension", currentExtension);
        telemetry.addData("- Current Voltage", currentVoltage);
        telemetry.addData("- Extension Error", extensionError);
        telemetry.addData("- Extension Target", extensionTarget);
        telemetry.addData("- Input Power", inputPower);
        telemetry.addData("- Last Voltage", lastVoltage);
        telemetry.addData("- Offset Voltage", offsetVoltage);
        telemetry.addData("- Output Power", outputPower);

    }

    // Sets the extension's position.
    public void setPosition(double position) {

        // Set the extension's position.
        this.extensionTarget = position;

    }

}