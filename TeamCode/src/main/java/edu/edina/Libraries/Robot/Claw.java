package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// This represents a claw.
@Config
public class Claw {

    // Closed position
    public static double CLOSED_POSITION = 0.48;

    // Open position
    public static double OPEN_POSITION = 0.05;

    // Open
    private boolean open;

    // Robot hardware
    private final RobotHardware robotHardware;

    // Servo
    private final Servo servo;

    // Initializes this.
    public Claw(RobotHardware robotHardware) {

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Get the claw servo.
        servo = hardwareMap.get(Servo.class, "claw_servo");

    }

    // Updates this.
    public void update() {

        // Get the claw's position.
        double position = servo.getPosition();

        // Get the op mode.
        LinearOpMode opMode = robotHardware.getOpMode();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Display claw telemetry.
        telemetry.addData("Claw", "====================");
        telemetry.addData("- Open", open);
        telemetry.addData("- Position", position);

    }

    // Toggles the claw.
    public void toggle() {

        // If the claw is open...
        if(open) {

            // Close the claw.
            close();

        }

        // Othewise (if the claw is cloded)...
        else {

            // Open the claw.
            open();

        }

    }

    // Closes the claw.
    public void close() {

        // Close the claw.
        servo.setPosition(CLOSED_POSITION);

        // Remember that the claw is closed.
        open = false;

    }

    // Opens the claw.
    public void open() {

        // Open the claw.
        servo.setPosition(OPEN_POSITION);

        // Remember that the claw is open.
        open = true;

    }

}
