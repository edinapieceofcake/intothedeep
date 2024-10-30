package edu.edina.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class DriveTrain {

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private LinearOpMode opMode;

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

        // Get hardware.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
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

        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.
        //leftBackPower = gamepad1.x ? 1.0 : 0.0;  // X gamepad
        //leftFrontPower = gamepad1.a ? 1.0 : 0.0;  // A gamepad
        //rightBackPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
        //rightFrontPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad


        // Verify inputs exist.
        if (leftFrontDrive == null) {
            throw new InterruptedException("The left front drive motor is missing.");
        }
        if (rightFrontDrive == null) {
            throw new InterruptedException("The right front drive motor is missing.");
        }
        if (leftBackDrive == null) {
            throw new InterruptedException("The left back drive motor is missing.");
        }
        if (rightBackDrive == null) {
            throw new InterruptedException("The right back drive motor is missing.");
        }

        // Move the robot.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

}