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
public class Swivel {
    public static double SWIVEL_SERVO_TRAVEL_TIME = 2.5;

    // Horizontal (BASED ON CLAW DIRECTION) position
    public static double HORIZONTAL_POSITION = 0.385;

    // Vertical (BASED ON CLAW DIRECTION) position
    public static double VERTICAL_POSITION = 0.65;

    // Clip (BASED ON CLAW DIRECTION) position
    public static double HIGH_CHAMBER_POSITION = 0.95;

    // Servo
    private final TrackingServo servo;

    private final Telemetry telemetry;

    // Horizontal
    private boolean horizontal = true;

    // Initializes this.
    public Swivel(HardwareMap hardwareMap, Telemetry telemetry) {
        this.servo = new TrackingServo(hardwareMap.get(Servo.class, "swivel"),
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
        servo.setPosition(HIGH_CHAMBER_POSITION);

    }

    // Sets the swivel to vertical.
    public void setVertical() {

        // Sets the swivel to vertical.
        servo.setPosition(VERTICAL_POSITION);

        // Remember that the swivel is vertical.
        horizontal = false;

    }

    public Action turnToWall() {
        return new SequentialAction(
                new InstantAction(() -> servo.setPosition(0.0822)),
                new WaitForTime(100)
        );
    }

    public Action turnToScore() {
        return new SequentialAction(
                new InstantAction(() -> servo.setPosition(0.6378)),
                new WaitForTime(100)
        );
    }

    public Action turnToPerpendicular() {
        return new SequentialAction(
                new InstantAction(() -> servo.setPosition(0.3611)),
                new WaitForTime(100)
        );
    }
}