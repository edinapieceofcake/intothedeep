package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// This represents a wrist.
@Config
public class Wrist {
    public static double WRIST_SERVO_TRAVEL_TIME = 2.5;

    // Down position
    public static double DOWN_POSITION = 0.03;

    public static double HIGH_CHAMBER_HOLD_POSITION = 0.58;
    public static double HIGH_CHAMBER_CLIP_POSITION = 0.85;

    public static double HIGH_BASKET_POSITION = 0.5;

    public static double LOW_CHAMBER_SCORE_POSITION = 0.00;

    // Up position
    public static double UP_POSITION = 0.6;

    // Init position
    public static double INITIALIZE_POSITION = 0.47;

    // Wall position
    public static double WALL_POSITION = 0.3;

    // Servo
    private final TrackingServo servo;

    private final Telemetry telemetry;

    // Up
    private boolean up;

    // Initializes this.
    public Wrist(HardwareMap hardwareMap, Telemetry telemetry) {
        this.servo = new TrackingServo(hardwareMap.get(Servo.class, "wrist"),
                WRIST_SERVO_TRAVEL_TIME);
        this.telemetry = telemetry;
    }

    public double getEstimatedPosition() {
        return servo.getEstimatedPosition();
    }

    // Updates this.
    public void update() {
        // Get the wrist's position.
        double position = servo.getEstimatedPosition();

        // Display wrist telemetry.
        telemetry.addData("Wrist", "====================");
        telemetry.addData("- Position", position);
        telemetry.addData("- Up", up);

    }

    // Toggles the wrist.
    public void toggle() {

        // If the wrist is up...
        if (up) {

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

    // Raises the wrist.
    public void initialize() {

        // Raise the wrist.
        servo.setPosition(INITIALIZE_POSITION);

        // Remember that the wrist is up.
        up = true;
    }

    // Set the wrist to wall position.
    public void setWallPosition() {

        // Sets the wrist to wall position.
        servo.setPosition(WALL_POSITION);

        // Remember that the wrist is up.
        up = true;
    }

    // Set the wrist to high basket position.
    public void setHighBasketPosition() {

        // Sets the wrist to high basket position.
        servo.setPosition(HIGH_BASKET_POSITION);

        // Remember that the wrist is up.
        up = true;
    }

    public void setHighChamberHoldPosition() {
        servo.setPosition(HIGH_CHAMBER_HOLD_POSITION);
    }
    public void setHighChamberClipPosition() {
        servo.setPosition(HIGH_CHAMBER_CLIP_POSITION);
    }

}
