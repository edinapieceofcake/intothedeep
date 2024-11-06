package edu.edina.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.edina.Libraries.Robot.RobotHardware;

/**
 * This represents a compound arm.
 */
@Config
public class CompoundArm {

    public static double CLAW_OPEN_POSITION = 0.56;
    public static double CLAW_CLOSED_POSITION = 0.77;
    public static double WRIST_DOWN_POSITION = 0;
    public static double WRIST_UP_POSITION = 0.3;
    public static int ARM_DOWN_POSITION = 0;
    public static int ARM_WALL_POSITION = 1000;
    public static int ARM_BASKET_POSITION = 2000;
    public static int ARM_CHAMBER_POSITION = 3000;
    public static double TICKS_PER_DEGREE = 23.3; // Determined experimentally
    private static final double INITIAL_DEGREES_BELOW_HORIZONTAL = 26; // Determined experimentally
    public static double P = 0.0005;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0.1;
    public static double RETRACT_SLIDE_POWER = 0.2;
    public static double EXTEND_SLIDE_POWER = -0.4;

    private int targetArmPosition = 0;
    private LinearOpMode opMode;
    private RobotHardware robotHardware;
    private FtcDashboard ftcDashboard;
    private boolean clawOpen;
    private boolean wristUp;
    private PIDController controller;

    /**
     * Initializes this.
     * @param opMode An op mode
     */
    public CompoundArm(LinearOpMode opMode) {

        // Remember the op mode.
        this.opMode = opMode;

        // Initialize the FTC dashboard.
        ftcDashboard = FtcDashboard.getInstance();

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Get hardware.
        robotHardware = new RobotHardware(hardwareMap);

        // Configure the arm motor.
        DcMotorEx armMotor = robotHardware.armMotor;
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize the arm controller.
        controller = new PIDController(P, I, D);

    }

    /**
     * Updates this.
     * @param telemetry A telemetry interface
     * @throws InterruptedException If updating this fails
     */
    public void update(Telemetry telemetry) throws InterruptedException {

        // Verify input exists.
        if (robotHardware.armMotor == null) {
            throw new InterruptedException("The arm motor is missing.");
        }
        if (robotHardware.liftMotorL == null) {
            throw new InterruptedException("The left lift motor is missing.");
        }
        if (robotHardware.liftMotorR == null) {
            throw new InterruptedException("The right lift motor is missing.");
        }
        if (robotHardware.claw == null) {
            throw new InterruptedException("The claw servo is missing.");
        }
        if (robotHardware.slideServo == null) {
            throw new InterruptedException("The slide servo is missing.");
        }
        if (robotHardware.slideEncoder == null) {
            throw new InterruptedException("The slide servo encoder is missing.");
        }
        if (robotHardware.wristLeft == null) {
            throw new InterruptedException("The left wrist servo is missing.");
        }
        if (robotHardware.wristRight == null) {
            throw new InterruptedException("The right wrist servo is missing.");
        }
        if (robotHardware.wristEncoderR == null) {
            throw new InterruptedException("The right wrist servo encoder is missing.");
        }
        if (robotHardware.wristEncoderL == null) {
            throw new InterruptedException("The left wrist servo encoder is missing.");
        }

        // Update the arm motor.
        //////////////////////////////////////////////////////////////////////

        // Get the arm motor.
        DcMotorEx armMotor = robotHardware.armMotor;

        // PIDF Loops & Arm Control | FTC | 16379 KookyBotz
        // https://www.youtube.com/watch?v=E6H6Nqe6qJo

        // Determine the appropriate arm power.
        controller.setPID(P, I, D);
        int actualArmPosition = armMotor.getCurrentPosition();
        double actualArmDegrees = getDegrees(actualArmPosition);
        double actualArmRadians = Math.toRadians(actualArmDegrees);
        double pid = controller.calculate(actualArmPosition, targetArmPosition);
        double targetArmDegrees = getDegrees(targetArmPosition);
        double feedForward = Math.cos(actualArmRadians) * F;
        double power = pid + feedForward;

        // Set the arm's power.
        armMotor.setPower(power);

        // Display arm telemetry.
        telemetry.addData("Arm", "====================");
        telemetry.addData("- PID", pid);
        telemetry.addData("- Feed Forward", feedForward);
        telemetry.addData("- Power", power);
        telemetry.addData("- Actual Position", actualArmPosition);
        telemetry.addData("- Actual Degrees", actualArmDegrees);
        telemetry.addData("- Target Position", targetArmPosition);
        telemetry.addData("- Target Degrees", targetArmDegrees);

        // Update the claw.
        //////////////////////////////////////////////////////////////////////

        // Get the claw.
        Servo claw = robotHardware.claw;

        // Get the claw's position.
        double clawPosition = claw.getPosition();

        // Display claw telemetry.
        telemetry.addData("Claw", "====================");
        telemetry.addData("- Open", clawOpen);
        telemetry.addData("- Position", clawPosition);

        // Update the slide.
        //////////////////////////////////////////////////////////////////////

        // Get the slide servo.
        CRServo slideServo = robotHardware.slideServo;

        // Get the slide encoder.
        AnalogInput slideEncoder = robotHardware.slideEncoder;

        // Get the slide's power.
        double slidePower = slideServo.getPower();

        // Get the slide's voltage.
        double slideVoltage = slideEncoder.getVoltage();

        // Display slide telemetry.
        telemetry.addData("Slide", "====================");
        telemetry.addData("- Power", slidePower);
        telemetry.addData("- Voltage", slideVoltage);

        // Update the wrist.
        //////////////////////////////////////////////////////////////////////

        // Get the wrist.
        Servo wrist = robotHardware.wristLeft;

        // Get the wrist's position.
        double wristPosition = wrist.getPosition();

        // Display wrist telemetry.
        telemetry.addData("Wrist", "====================");
        telemetry.addData("- Up", wristUp);
        telemetry.addData("- Position", wristPosition);

    }

    /**
     * Toggles the wrist.
     */
    public void toggleWrist() {
        if(wristUp) {
            lowerWrist();
        }
        else {
            raiseWrist();
        }
    }

    /**
     * Lowers the wrist.
     */
    private void lowerWrist() {
        Servo wrist = robotHardware.wristLeft;
        wrist.setPosition(WRIST_DOWN_POSITION);
        wristUp = false;
    }

    /**
     * Raises the wrist.
     */
    private void raiseWrist() {
        Servo wrist = robotHardware.wristLeft;
        wrist.setPosition(WRIST_UP_POSITION);
        wristUp = true;
    }

    /**
     * Sets the arm position.
     * @param position A position
     */
    public void setArmPosition(int position) {
        targetArmPosition = position;
    }

    /**
     * Converts the arm motor's ticks to degrees.
     * @param ticks A tick count
     * @return A degrees value
     */
    private double getDegrees(double ticks) {
        double degrees = ticks / TICKS_PER_DEGREE - INITIAL_DEGREES_BELOW_HORIZONTAL;
        return degrees;
    }

    /**
     * Toggles the claw.
     */
    public void toggleClaw() {
        if(clawOpen) {
            closeClaw();
        }
        else {
            openClaw();
        }
    }

    /**
     * Closes the claw.
     */
    private void closeClaw() {
        Servo claw = robotHardware.claw;
        claw.setPosition(CLAW_CLOSED_POSITION);
        clawOpen = false;
    }

    /**
     * Opens the claw.
     */
    private void openClaw() {
        Servo claw = robotHardware.claw;
        claw.setPosition(CLAW_OPEN_POSITION);
        clawOpen = true;
    }

    /**
     * Retracts the slide
     */
    public void retractSlide() {
        CRServo slideServo = robotHardware.slideServo;
        slideServo.setPower(RETRACT_SLIDE_POWER);
    }

    /**
     * Extends the slide.
     */
    public void extendSlide() {
        CRServo slideServo = robotHardware.slideServo;
        slideServo.setPower(EXTEND_SLIDE_POWER);
    }

    /**
     * Stops the slide.
     */
    public void stopSlide() {
        CRServo slideServo = robotHardware.slideServo;
        slideServo.setPower(0);
    }

}