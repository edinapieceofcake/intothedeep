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

    // Baseket position
    public static double BASKET_POSITION = 0.70;

    // Chamber position
    public static double CHAMBER_POSITION = 0.82;

    // Submersible position
    public static double SUBMERSIBLE_POSITION = 0.16;

    // Wall position
    public static double WALL_POSITION = 0.47;

    // Servo
    private final TrackingServo servo;

    private final Servo servo2;

    private final Servo wrist;

    private final Telemetry telemetry;

    // Initializes this.
    public Wrist(HardwareMap hardwareMap, Telemetry telemetry) {
        servo2 = hardwareMap.get(Servo.class, "wrist");
        this.servo = new TrackingServo(servo2, WRIST_SERVO_TRAVEL_TIME);
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

    }

    // Set the wrist to wall position.
    public void setWallPosition() {

        // Sets the wrist to wall position.
        servo.setPosition(WALL_POSITION);

    }

    // Set the wrist to basket position.
    public void setBasketPosition() {

        // Sets the wrist to basket position.
        servo.setPosition(BASKET_POSITION);

    }

    // Set the wrist to submersible position.
    public void setSubmersiblePosition() {

        // Sets the wrist to submersible position.
        servo.setPosition(SUBMERSIBLE_POSITION);

    }

    // Set the wrist to the chamber position.
    public void setChamberPosition() {

        // Set the wrist to the chamber position.
        servo.setPosition(CHAMBER_POSITION);

    }

    // Determines whether the wrist is in the wall position.
    public boolean isInWallPosition() {

        // Return the result.
        return Math.abs(servo2.getPosition() - WALL_POSITION) < 0.0001;

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