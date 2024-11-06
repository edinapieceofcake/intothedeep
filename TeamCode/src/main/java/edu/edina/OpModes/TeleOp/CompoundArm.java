package edu.edina.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.edina.Libraries.Robot.RobotHardware;

@Config
public class CompoundArm {
    public static double CLAW_OPEN_POSITION = 0.56;
    public static double CLAW_CLOSED_POSITION = 0.77;
    public static double WRIST_DOWN_POSITION = 0;
    public static double WRIST_UP_POSITION = 0.3;
    public static int ARM_DOWN_POSITION = 0;
    public static int ARM_WALL_POSITION = 200;
    public static int ARM_BASKET_POSITION = 400;
    public static int ARM_CHAMBER_POSITION = 600;
    public static double TICKS_PER_DEGREE = 23.3; // Determined experimentally
    private static final double INITIAL_DEGREES_BELOW_HORIZONTAL = 26; // Determined experimentally
    public static double P = 0.0005;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0.1;
    public static int TARGET_ARM_POSITION = 0;

    private LinearOpMode opMode;
    private RobotHardware robotHardware;
    private FtcDashboard ftcDashboard;
    private boolean clawOpen;
    private boolean wristUp;
    private PIDController controller;
    double clawPosition;
    double wristLeftPos;

    public CompoundArm(LinearOpMode opMode) {

        // Remember the op mode.
        this.opMode = opMode;

        // Initialize the FTC dashboard.
        ftcDashboard = FtcDashboard.getInstance();

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Get hardware.
        robotHardware = new RobotHardware(hardwareMap);

        robotHardware.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robotHardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller = new PIDController(P, I, D);

    }

    public void update(Telemetry telemetry) throws InterruptedException {

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

        DcMotorEx armMotor = robotHardware.armMotor;

        // PIDF Loops & Arm Control | FTC | 16379 KookyBotz
        // https://www.youtube.com/watch?v=E6H6Nqe6qJo

        controller.setPID(P, I, D);
        int actualArmPosition = armMotor.getCurrentPosition();
        double actualArmDegrees = getDegrees(actualArmPosition);
        double actualArmRadians = Math.toRadians(actualArmDegrees);
        double pid = controller.calculate(actualArmPosition, TARGET_ARM_POSITION);
        double targetArmDegrees = getDegrees(TARGET_ARM_POSITION);
        double feedForward = Math.cos(actualArmRadians) * F;

        double power = pid + feedForward;

        armMotor.setPower(power);

        telemetry.addData("Status", "Running");
        telemetry.addData("PID", pid);
        telemetry.addData("Feed Forward", feedForward);
        telemetry.addData("Power", power);
        telemetry.addData("Actual Arm Position", actualArmPosition);
        telemetry.addData("Actual Arm Degrees", actualArmDegrees);
        telemetry.addData("Target Arm Position", TARGET_ARM_POSITION);
        telemetry.addData("Target Arm Degrees", targetArmDegrees);

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
        robotHardware.armMotor.setTargetPosition(position);
    }

    private double getDegrees(double ticks) {
        double degrees = ticks / TICKS_PER_DEGREE - INITIAL_DEGREES_BELOW_HORIZONTAL;
        return degrees;
    }

}