package edu.edina.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import edu.edina.Libraries.Robot.RobotHardware;

@Config
public class CompoundArm {
    public static double CLAW_OPEN_POSITION = 0.56;
    public static double CLAW_CLOSED_POSITION = 0.77;
    public static double WRIST_DOWN_POSITION = 0;
    public static double WRIST_UP_POSITION = 0.3;
    public static int ARM_DOWN_POSITION = 0; // Position 1
    public static int ARM_WALL_POSITION = 100; // Position 2
    public static int ARM_BASKET_POSITION = 200; // Position 3
    public static int ARM_CHAMBER_POSITION = 300; // Position 4
    public static double ARM_MOTOR_SPEED = 0.1;
    public static double P = 10.0;
    public static double I = 3.0;
    public static double D = 0.0;
    public static double F = 0.0;
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
    int armMotorPosition;

    public CompoundArm(LinearOpMode opMode) throws InterruptedException {
        // Remember the op mode.
        this.opMode = opMode;

        // Initialize the FTC dashboard.
        ftcDashboard = FtcDashboard.getInstance();

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Get hardware.
        robotHardware = new RobotHardware(hardwareMap);

        robotHardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware.armMotor.setTargetPosition(0);
        robotHardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardware.armMotor.setPower(ARM_MOTOR_SPEED);
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

        PIDFCoefficients armMotorPIDFCoefficients = new PIDFCoefficients(P, I, D, F);
        robotHardware.armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, armMotorPIDFCoefficients);
    }

    public void toggleClaw() {
        clawPosition = clawOpen ? CLAW_CLOSED_POSITION : CLAW_OPEN_POSITION;
        clawOpen = !clawOpen;
        robotHardware.claw.setPosition(clawPosition);
    }

    public void toggleWrist() {
        wristLeftPos = wristUp ? WRIST_DOWN_POSITION : WRIST_UP_POSITION;
        wristUp = !wristUp;
        robotHardware.wristLeft.setPosition(wristLeftPos);
    }

    public void setArmPosition(int position) {
        if (position == 1) {
            armMotorPosition = ARM_DOWN_POSITION;
        }
        if (position == 2) {
            armMotorPosition = ARM_WALL_POSITION;
        }
        if (position == 3) {
            armMotorPosition = ARM_BASKET_POSITION;
        }
        if (position == 4) {
            armMotorPosition = ARM_CHAMBER_POSITION;
        }
        robotHardware.armMotor.setTargetPosition(armMotorPosition);
    }

    public PIDFCoefficients getPID() {
        PIDFCoefficients armMotorPID = robotHardware.armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        return armMotorPID;
    }

    public double getArmEncoder() {
        return robotHardware.armMotor.getCurrentPosition();
    }
}