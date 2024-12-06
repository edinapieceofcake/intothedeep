package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// This represents a wrist.
@Config
public class Swivel {
    public static double SWIVEL_SERVO_TRAVEL_TIME = 2.5;

    // Horizontal (BASED ON CLAW DIRECTION) position
    public static double HORIZONTAL_POSITION = 0;

    // Vertical (BASED ON CLAW DIRECTION) position
    public static double VERTICAL_POSITION = 0;

    // Clip (BASED ON CLAW DIRECTION) position
    public static double CLIP_POSITION = 0;

    // Servo
    private final TrackingServo servo;

    private final Telemetry telemetry;

    // Horizontal
    private boolean horizontal = true;

    // Initializes this.
    public Swivel(HardwareMap hardwareMap, Telemetry telemetry) {
        this.servo = new TrackingServo(hardwareMap.get(Servo.class, "wrist_left"),
                SWIVEL_SERVO_TRAVEL_TIME);
        this.telemetry = telemetry;
    }

    public double getEstimatedPosition() {
        return servo.getEstimatedPosition();
    }

    // Updates this.
    public void update() {
        // Get the swivel's position.
        double position = servo.getEstimatedPosition();

        // Display swivel telemetry.
        telemetry.addData("Swivel", "====================");
        telemetry.addData("- Position", position);
        telemetry.addData("- Horizontal", horizontal);

    }

    // Toggles the swivel.
    public void toggle() {

        // If the swivel is horizontal...
        if (horizontal) {

            // Set the swivel to vertical.
            setVertical();

        }

        // Otherwise (if the swivel is vertical)...
        else {

            // Set the swivel to horizontal.
            setHorizontal();

        }

    }

    // Sets the swivel to horizontal.
    public void setHorizontal() {

        // Sets the swivel to horizontal.
        servo.setPosition(HORIZONTAL_POSITION);

        // Remember that the swivel is horizontal.
        horizontal = true;

    }

    // Sets the swivel to clip.
    public void setClip() {

        // Sets the swivel to clip.
        servo.setPosition(CLIP_POSITION);

    }

    // Sets the swivel to vertical.
    public void setVertical() {

        // Sets the swivel to vertical.
        servo.setPosition(VERTICAL_POSITION);

        // Remember that the swivel is vertical.
        horizontal = false;

    }

}
