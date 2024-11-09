package edu.edina.Libraries.Robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.RoadRunner.ThreeDeadWheelLocalizer;

public class RobotHardware {
    /*
    Control Hub Portal
        Control Hub
            Motors
                0 - GoBILDA 5201 series - right_lift_motor (has right lift encoder)
                1 - GoBILDA 5201 series - front_encoder (is front odometry encoder)
                2 - GoBILDA 5201 series - left_back_drive (encoder port returns 0 and -1)
                3 - GoBILDA 5201 series - left_front_drive (has left odometry encoder)
            Servos
                0 - Axon Micro+ ServoE - wrist_left
                1 - GoBilda 5 turn - claw_servo
                2 - Axon Micro+ ServoF - wrist_right
            Digital Devices
                5 - REV Touch Sensor - arm_touch
                7 - REV Touch Sensor - lift_touch
            I2C
                3 - Neopixel Driver - neopixel_driver
            Analog
                0 - Axon Micro+ ServoE - wrist_left_encoder
                1 - Axon Micro+ ServoF - wrist_right_encoder
        Expansion Hub 2
            Motors
                0 - GoBILDA 5201 series - left_lift_motor (has left lift encoder)
                1 - GoBILDA 5201 series - arm_motor (has through bore encoder)
                2 - GoBILDA 5201 series - right_front_drive (encoder port has bent pin)
                3 - GoBILDA 5201 series - right_back_drive (has right odometry encoder)
            Servos
                0 - CRServo Axon Mini+ - slide_servo
            Analog
                0 - Axon Mini+ Encoder - slide_encoder
    */

    private final LinearOpMode opMode;
    public final IMU imu;
    public ThreeDeadWheelLocalizer odometry;
    public final MecanumDrive drive;
    public final VoltageSensor voltageSensor;
    private Drivetrain drivetrain;
    private final Wrist wrist;
    private final Arm arm;
    private final Claw claw;
    private final Lift lift;
    private final Slide slide;

    public RobotHardware(LinearOpMode opMode) throws InterruptedException {

        // Remember the op mode.
        this.opMode = opMode;

        // Initialize the arm.
        arm = new Arm(this);

        // Initialize the claw.
        claw = new Claw(this);

        // Initialize the lift.
        lift = new Lift(this);

        // Initialize the slide.
        slide = new Slide(this);

        // Initialize the wrist.
        wrist = new Wrist(this);

        HardwareMap hardwareMap = opMode.hardwareMap;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        odometry = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        // Initialize the drivetrain.
        drivetrain = new Drivetrain(opMode);

    }

    // Waits for the user to lower the lift.
    public void waitForLiftDown() {

        // Waits for the user to lower the lift.
        lift.waitForDown();

    }

    // Waits for the user to lower the arm.
    public void waitForArmDown() throws InterruptedException {

        // Waits for the user to lower the arm.
        arm.waitForDown();

    }

    // Logs a message.
    public void log(String message) {

        // If the op mode is missing...
        if (opMode == null) {

            // Exit the method.
            return;

        }

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Show the message.
        telemetry.addData("Message", message);
        telemetry.update();

    }

    // Initializes the robot.
    public void initializeRobot() {

        // Move the arm to the ground position.
        arm.setGroundPosition();

        // Close the claw.
        claw.close();

        // Lower the wrist.
        wrist.lower();

    }

    // Toggles the wrist.
    public void toggleWrist() {

        // Toggle the wrist.
        wrist.toggle();

    }

    // Raises the wrist.
    public void raiseWrist() {

        // Raise the wrist.
        wrist.raise();

    }

    // Lowers the wrist.
    public void lowerWrist() {

        // Lowers the wrist.
        wrist.lower();

    }

    // Updates this.
    public void update() {

        // Update the arm.
        arm.update();

        // Update the claw.
        claw.update();

        // Update the drivetrain.
        drivetrain.update();

        // Update the lift.
        lift.update();

        // Update the slide.
        slide.update();

        // Update the wrist.
        wrist.update();

    }

    // Goes to the previous arm position.
    public void previousArmPosition() {

        // Go to the previous arm position.
        arm.previousPosition();

    }

    // Goes to the next arm position.
    public void nextArmPosition() {

        // Go to the next arm position.
        arm.nextPosition();

    }

    // Decrements the arm position.
    public void decrementArmPosition() {

        // Decrement the arm position.
        arm.decrementPosition();

    }

    // Increments the arm position.
    public void incrementArmPosition() {

        // Increment the arm position.
        arm.incrementPosition();

    }

    // Toggles the claw.
    public void toggleClaw() {

        // Toggle the claw.
        claw.toggle();

    }

    // Raises the lift.
    public void raiseLift() {

        // Raise the lift.
        lift.raise();

    }

    // Lowers the lift.
    public void lowerLift() {

        // Lower the lift.
        lift.lower();

    }

    // Gets the op mode.
    public LinearOpMode getOpMode() {

        // Return the op mode.
        return opMode;

    }

    // Sets the slide's position.
    public void setSlidePosition(double position) {

        // Set the slide's position.
        slide.setPosition(position);

    }

    // Retracts the slide.
    public void retractSlide() {

        // Retract the slide.
        slide.retract();

    }

    // Extends the slide.
    public void extendSlide() {

        // Extend the slide.
        slide.extend();

    }

    // Sets the minimum extension.
    public void setMinimumExtension() {

        // Set the minimum extension.
        slide.setMinimumExtension();

    }

    // Sets the low basket extension.
    public void setLowBasketExtension() {

        // Set the low basket extension.
        slide.setLowBasketExtension();

    }

    // Sets the high basket extension.
    public void setHighBasketExtension() {

        // Set the high basket extension.
        slide.setHighBasketExtension();

    }

    // Toggles turtle mode.
    public void toggleTurtleMode() {

        // Toggle turtle mode.
        drivetrain.toggleTurtleMode();

    }

    // Determines whether the arm is nearly down.
    public boolean isArmNearlyDown() {

        // Determine whether the arm is nearly down.
        boolean isNearlyDown = arm.isNearlyDown();

        // Return the result.
        return isNearlyDown;

    }

    // Moves the arm to the ground position.
    public void setArmGroundPosition() {

        // Move the arm to the ground position.
        arm.setGroundPosition();

    }

    // Moves the arm to the low basket position.
    public void setArmLowBasketPosition() {

        // Move the arm to the low basket position.
        arm.setLowBasketPosition();

    }

    // Moves the arm to the high basket position.
    public void setArmHighBasketPosition() {

        // Move the arm to the high basket position.
        arm.setHighBasketPosition();

    }

    // Moves the arm to the high chamber position.
    public void setArmHighChamberPosition() {

        // Move the arm to the high chamber position.
        arm.setHighChamberPosition();

    }

    // Moves the arm to the low chamber position.
    public void setArmLowChamberPosition() {

        // Move the arm to the low chamber position.
        arm.setLowChamberPosition();

    }

    // Moves the arm to the submersible position.
    public void setArmSubmersiblePosition() {

        // Move the arm to the submersible position.
        arm.setSubmersiblePosition();

    }

    // Moves the lift to the ground position.
    public void setLiftGroundPosition() {

        // Move the lift to the ground position.
        lift.setGroundPosition();

    }

    // Moves the lift to the high basket position.
    public void setLiftHighBasketPosition() {

        // Move the lift to the high basket position.
        lift.setHighBasketPosition();

    }

}