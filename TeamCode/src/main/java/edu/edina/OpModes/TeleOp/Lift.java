package edu.edina.OpModes.TeleOp;

import android.text.method.Touch;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.edina.Libraries.Robot.RobotHardware;

@Config
public class Lift {

    public static int DOWN_POSITION = 0;
    public static double LOWER_POWER = 0.6;
    public static double RAISE_POWER = 1;
    public static int THRESHOLD = 50;

    private RobotHardware robotHardware;
    private int targetPosition;
    private LinearOpMode opMode;

    // Initializes this.
    public Lift(LinearOpMode opMode, RobotHardware robotHardware) {

        // Remember the robot hardware.
        this.robotHardware = robotHardware;

        // Remember the op mode.
        this.opMode = opMode;

    }

    // Update this.
    public void update() {

        // Get the lift's position.
        DcMotorEx leftMotor = robotHardware.liftMotorLeft;
        DcMotorEx rightMotor = robotHardware.liftMotorRight;
        int leftPosition = leftMotor.getCurrentPosition();
        int rightPosition = rightMotor.getCurrentPosition();

        TouchSensor touch = robotHardware.liftTouch;
        // If we finished lowering the lift...
        if(targetPosition == DOWN_POSITION && touch.isPressed()) {

            // Reset the lift.
            reset();

        }

        // Get the lift's power.
        double leftPower = leftMotor.getPower();
        double rightPower = rightMotor.getPower();

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Add lift information to the telemetry.
        telemetry.addData("Lift", "Position = %d/%d, Power = %.2f/%.2f", leftPosition, rightPosition, leftPower, rightPower);

    }

    // Resets the lift.
    private void reset() {
        DcMotorEx leftMotor = robotHardware.liftMotorLeft;
        DcMotorEx rightMotor = robotHardware.liftMotorRight;

        // Reset the lift motors.
        resetMotor(leftMotor);
        resetMotor(rightMotor);

    }

    // Resets a lift motor.
    private static void resetMotor(DcMotor liftMotor) {

        // Reset the lift motor.
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void setPower(double power) {
        DcMotorEx leftMotor = robotHardware.liftMotorLeft;
        DcMotorEx rightMotor = robotHardware.liftMotorRight;
        setMotorPower(leftMotor, power);
        setMotorPower(rightMotor, power);
    }

    private static void setMotorPower(DcMotorEx motor, double power) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(power);
    }

    // Sets the lift's position.
    public void setPosition(int targetPosition) {

        DcMotorEx leftMotor = robotHardware.liftMotorLeft;
        DcMotorEx rightMotor = robotHardware.liftMotorRight;

        // Remember the target position.
        this.targetPosition = targetPosition;

        // Get the lift's current position.
        int currentPosition = leftMotor.getCurrentPosition();

        // Get the appropriate power.
        double power = targetPosition > currentPosition ? RAISE_POWER : LOWER_POWER;

        // Lower the lift.
        setPosition(leftMotor, targetPosition, power);
        setPosition(rightMotor, targetPosition, power);

    }

    // Sets the lift position.
    private void setPosition(DcMotor liftMotor, int position, double power) {

        // Set the lift position.
        liftMotor.setTargetPosition(position);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(power);

    }

    public boolean isInPosition(int targetPosition) {

        DcMotorEx leftMotor = robotHardware.liftMotorLeft;
        DcMotorEx rightMotor = robotHardware.liftMotorRight;

        int leftPosition = leftMotor.getCurrentPosition();
        int rightPosition = rightMotor.getCurrentPosition();
        int leftDifference = Math.abs(leftPosition - targetPosition);
        int rightDifference = Math.abs(rightPosition - targetPosition);
        if (leftDifference < THRESHOLD && rightDifference < THRESHOLD) {
            return true;
        }
        else {
            return false;
        }
    }

    public int getLiftPosition() {
        DcMotorEx leftMotor = robotHardware.liftMotorLeft;
        return leftMotor.getCurrentPosition();
    }

}
