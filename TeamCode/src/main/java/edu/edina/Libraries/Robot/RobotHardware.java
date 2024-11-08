package edu.edina.Libraries.Robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.RoadRunner.ThreeDeadWheelLocalizer;
import edu.edina.OpModes.TeleOp.CompoundArm;
import edu.edina.OpModes.TeleOp.DriveTrain;

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
    public final DcMotorEx leftFrontDrive, rightFrontDrive, rightBackDrive, leftBackDrive;
    public final DcMotorEx liftMotorLeft, liftMotorRight;
    public final CRServo slideServo;
    public final AnalogInput slideEncoder;
    public final IMU imu;
    public ThreeDeadWheelLocalizer odometry;
    public final MecanumDrive drive;
    public final VoltageSensor voltageSensor;
    public final TouchSensor liftTouch;
    public DriveTrain driveTrain;
    public CompoundArm compoundArm;
    private final Wrist wrist;
    private final Arm arm;
    private final Claw claw;

    public RobotHardware(LinearOpMode opMode) throws InterruptedException {

        // Remember the op mode.
        this.opMode = opMode;

        // Initialize the arm.
        arm = new Arm(opMode, this);

        // Initialize the claw.
        claw = new Claw(opMode);

        // Initialize the wrist.
        wrist = new Wrist(opMode);

        HardwareMap hardwareMap = opMode.hardwareMap;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        odometry = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick);

        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotorLeft = hardwareMap.get(DcMotorEx.class, "left_lift_motor");
        liftMotorRight = hardwareMap.get(DcMotorEx.class, "right_lift_motor");
        liftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        slideServo = hardwareMap.get(CRServo.class, "slide_servo");

        liftTouch = hardwareMap.get(TouchSensor.class, "lift_touch");

        slideEncoder = hardwareMap.get(AnalogInput.class, "slide_encoder");

        // Get the robot drive train.
        driveTrain = new DriveTrain(opMode, this);

        // Get the robot compound arm
        compoundArm = new CompoundArm(opMode, this);
    }

    public double getSlideVoltage() {
        return slideEncoder.getVoltage();
    }

    public int getLeftLift() {
        return liftMotorLeft.getCurrentPosition();
    }

    public int getRightLift() {
        return liftMotorRight.getCurrentPosition();
    }

    // Waits for the user to lower the lift.
    public void waitForLiftDown() throws InterruptedException {

        // While the lift is up...
        while (!opMode.isStopRequested() && !liftTouch.isPressed()) {

            // Instruct the user to lower the lift.
            log("Please lower the lift...");

        }

        // Reset the lift.
        resetLift();

        // Notify the user that the lift is down.
        log("Lift is down");

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

    // Resets the lift.
    private void resetLift() throws InterruptedException {

        // Verify inputs exist.
        if(liftMotorLeft == null) {
            throw new InterruptedException("The left lift motor is missing.");
        }
        if(liftMotorRight == null) {
            throw new InterruptedException("The right lift motor is missing.");
        }

        // Reset the lift.
        resetLift(liftMotorLeft);
        resetLift(liftMotorRight);

    }

    // Resets a lift motor.
    private static void resetLift(DcMotor liftMotor) throws InterruptedException {

        // Verify inputs exist.
        if(liftMotor == null) {
            throw new InterruptedException("The lift motor is missing.");
        }

        // Reset the lift motor.
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    // Initializes the robot.
    public void initializeRobot() {

        // Move the arm to its first position.
        arm.setFirstPosition();

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

    // Updates this.
    public void update() {

        // Update the arm.
        arm.update();

        // Update the claw.
        claw.update();

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

}