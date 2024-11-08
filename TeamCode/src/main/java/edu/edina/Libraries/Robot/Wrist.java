package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// This represents a wrist.
@Config
public class Wrist {

    // Down position
    public static double DOWN_POSITION = 0.07;

    // Up position
    public static double UP_POSITION = 0.4;

    // Op mode
    private final LinearOpMode opMode;

    // Servo
    public final Servo servo;

    // Up
    private boolean up;

    // Initializes this.
    public Wrist(LinearOpMode opMode) {

        // Remember the op mode.
        this.opMode = opMode;

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Get the wrist servo.
        servo = hardwareMap.get(Servo.class, "wrist_left");

    }

    // Updates this.
    public void update() {

        // Get the wrist's position.
        double position = servo.getPosition();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Display wrist telemetry.
        telemetry.addData("Wrist", "====================");
        telemetry.addData("- Position", position);
        telemetry.addData("- Up", up);

    }

    // Toggles the wrist.
    public void toggle() {

        // If the wrist is up...
        if(up) {

            // Lower the wrist.
            lower();

        }

        // Otherwise (if the wrist is down)...
        else {

            // Raise the wrist.
            raise();

        }

    }

    // Lowers the wrist.
    public void lower() {

        // Lower the wrist.
        servo.setPosition(DOWN_POSITION);

        // Remember that the wrist is down.
        up = false;

    }

    // Raises the wrist.
    public void raise() {

        // Raise the wrist.
        servo.setPosition(UP_POSITION);

        // Remember that the wrist is up.
        up = true;

    }

}
