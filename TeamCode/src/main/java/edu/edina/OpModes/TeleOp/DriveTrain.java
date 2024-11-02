package edu.edina.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import edu.edina.Libraries.Robot.RobotHardware;


public class DriveTrain {
    private LinearOpMode opMode;
    private RobotHardware robotHardware;

    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;

    double max;

    public DriveTrain(LinearOpMode opMode) throws InterruptedException {
        // Remember the op mode.
        this.opMode = opMode;

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        //get hardware
        robotHardware = new RobotHardware(hardwareMap);
    }

    public void update() throws InterruptedException {
        Gamepad currentGamepad = opMode.gamepad1;

        double axial = -currentGamepad.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = currentGamepad.left_stick_x;
        double yaw = currentGamepad.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        leftFrontPower = axial + lateral + yaw;
        rightFrontPower = axial - lateral - yaw;
        leftBackPower = axial - lateral + yaw;
        rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        double multiplier;

        /*
        ***TURTLE MODE CODE (need to initialize variable)***

        if (isTurtleMode) {
            multiplier = TURTLE_MULTIPLIER;
        } else {
            multiplier = NORMAL_MULTIPLIER;
        }

        leftFrontPower *= multiplier;
        leftBackPower *= multiplier;
        rightBackPower *= multiplier;
        rightFrontPower *= multiplier;
        */

        // Verify inputs exist.
        if (robotHardware.leftFrontDrive == null) {
            throw new InterruptedException("The left front drive motor is missing.");
        }
        if (robotHardware.rightFrontDrive == null) {
            throw new InterruptedException("The right front drive motor is missing.");
        }
        if (robotHardware.leftBackDrive == null) {
            throw new InterruptedException("The left back drive motor is missing.");
        }
        if (robotHardware.rightBackDrive == null) {
            throw new InterruptedException("The right back drive motor is missing.");
        }

        // Move the robot.
        robotHardware.leftFrontDrive.setPower(leftFrontPower);
        robotHardware.rightFrontDrive.setPower(rightFrontPower);
        robotHardware.leftBackDrive.setPower(leftBackPower);
        robotHardware.rightBackDrive.setPower(rightBackPower);
    }
}