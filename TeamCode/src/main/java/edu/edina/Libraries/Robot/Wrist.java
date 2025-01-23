package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// This represents a wrist.
@Config
public class Wrist {
    public static double WRIST_SERVO_TRAVEL_TIME = 2.5;

    // Down position
    public static double DOWN_POSITION = 0.75;

    public static double CHAMBER_POSITION = 0.8194;

    public static double BASKET_POSITION = 0.7;

    // Up position
    public static double UP_POSITION = 0.1583;

    // Wall position
    //0.41
    public static double WALL_POSITION = 0.47;

    // Servo
    private final TrackingServo servo;

    private final Servo wrist;

    private final Telemetry telemetry;

    // Up
    private boolean up;

    // Initializes this.
    public Wrist(HardwareMap hardwareMap, Telemetry telemetry) {
        this.servo = new TrackingServo(hardwareMap.get(Servo.class, "wrist"),
                WRIST_SERVO_TRAVEL_TIME);
        wrist = hardwareMap.get(Servo.class, "wrist");
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

    // Angles the wrist.
//    public void angle() {
//
//        // Raise the wrist.
//        servo.setPosition(ANGLE_POSITION);
//
//        // Remember that the wrist is up.
//        up = true;
//    }

    // Raises the wrist.
    public void initialize() {

        // Raise the wrist.
        servo.setPosition(DOWN_POSITION);

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

    // Set the wrist to basket position.
    public void setBasketPosition() {

        // Sets the wrist to basket position.
        servo.setPosition(BASKET_POSITION);

        // Remember that the wrist is up.
        up = true;
    }

    public void setChamberPosition() {
        servo.setPosition(CHAMBER_POSITION);
    }


    public Action submersibleGrab() {
        return new SequentialAction(
                new InstantAction(() -> wrist.setPosition(0.1583)),
                new WaitForTime(200)
        );
    }

    public Action scorePosition() {
        return new SequentialAction(
                new InstantAction(() -> wrist.setPosition(0.8194)),
                new WaitForTime(200)
        );
    }

    public Action wallPosition() {
        return new SequentialAction(
                new InstantAction(() -> wrist.setPosition(0.3844)),
                new WaitForTime(200)
        );
    }
}