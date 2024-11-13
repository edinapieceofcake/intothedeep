package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Slide {

    // Low basket extension
    public static double LOW_BASKET_EXTENSION = 0.6;

    // High basket extension
    public static double HIGH_BASKET_EXTENSION = 0.9;

    // Maximum extension
    public static double MAXIMUM_EXTENSION = .95;

    // Minimum extension
    public static double MINIMUM_EXTENSION = .48;

    // Robot hardware
    private final RobotHardware robotHardware;

    // Servo
    private final Servo servo;

    // Initializes this.
    public Slide(RobotHardware robotHardware) {

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Get the servo.
        servo = hardwareMap.get(Servo.class, "slide_servo");

    }

    // Updates this.
    public void update() {

        // Get the servo's position.
        double position = servo.getPosition();

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Update the telemetry.
        telemetry.addData("Slide", "====================");
        telemetry.addData("- Position", position);

    }

    // Gets the exten
    public double getPosition () {
        return servo.getPosition();
    }

    // Sets the slide's position.
    public void setPosition(double position) {

        // Set the slide's position.
        servo.setPosition(position);

    }

    // Sets the minimum extension.
    public void setMinimumExtension() {

        // Set the minimum extension.
        servo.setPosition(MINIMUM_EXTENSION);

    }

    // Sets the maximum extension.
    public void setMaximumExtension() {

        // Set the maximum extension.
        servo.setPosition(MAXIMUM_EXTENSION);

    }

    // Sets the low basket extension.
    public void setLowBasketExtension() {

        // Set the low basket extension.
        servo.setPosition(LOW_BASKET_EXTENSION);

    }

    // Sets the high basket extension.
    public void setHighBasketExtension() {

        // Set the high basket extension.
        servo.setPosition(HIGH_BASKET_EXTENSION);

    }

}