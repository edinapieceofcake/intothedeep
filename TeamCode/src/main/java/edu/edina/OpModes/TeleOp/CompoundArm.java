package edu.edina.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import edu.edina.Libraries.Robot.RobotHardware;

@Config
public class CompoundArm {
    public static double CLAW_OPEN_POSITION = 0.56;
    public static double CLAW_CLOSED_POSITION = 0.77;
    public static double WRIST_DOWN_POSITION = 0;
    public static double WRIST_UP_POSITION = 1;
    private LinearOpMode opMode;
    private RobotHardware robotHardware;
    private FtcDashboard ftcDashboard;
    private boolean clawOpen;
    private boolean wristUp;

    double armMotorPower;
    double leftLiftPower;
    double rightLiftPower;
    double clawPosition;
    double slidePower;
    double slideEncoder;
    double wristLeftPower;
    double wristRightPower;
    double wristLeftPos;
    double wristRightPos;

    public CompoundArm(LinearOpMode opMode) throws InterruptedException {
        // Remember the op mode.
        this.opMode = opMode;

        // Initialize the FTC dashboard.
        ftcDashboard = FtcDashboard.getInstance();

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Get hardware.
        robotHardware = new RobotHardware(hardwareMap);
    }

    public void update() throws InterruptedException {
        Gamepad currentGamepad = opMode.gamepad1;

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

        /*armMotorPower;
        leftLiftPower;
        rightLiftPower;
        clawPosition;
        slidePower;
        slideEncoder;
        wristLeftPower;
        wristRightPower;
        wristLeftPos;
        wristRightPos;*/

        // Check gamepad.
            // Set power variables.

        // Turtle mode multipliers.

        // Call set power.
        robotHardware.claw.setPosition(clawPosition);

    }

    public void toggleClaw() {
        clawPosition = clawOpen ? CLAW_CLOSED_POSITION : CLAW_OPEN_POSITION;
        clawOpen = clawOpen ? false : true;
    }
}