package edu.edina.OpModes.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import edu.edina.Libraries.Robot.RobotHardware;

@Config
@TeleOp
public class TeleOpForScrimmage extends LinearOpMode {

    /*

    Robot Controls

    left stick = move robot
    right stick = rotate robot

    a = toggle claw
    x = toggle wrist
    y = toggle turtle mode

    dpad up = next arm position
    dpad down = previous arm position
    dpad right = increment arm position
    dpad left = decrement arm position

    right trigger = raise lift
    left trigger = lower lift

    left bumper = retract slide
    right bumper = extend slide

    */

    // Trigger threshold
    private static final double TRIGGER_THRESHOLD = 0.5;

    // Runs the op mode.
    public void runOpMode() throws InterruptedException {

        // Get hardware.
        RobotHardware robotHardware = new RobotHardware(this);

        // Wait for the user to lower the lift.
        robotHardware.waitForLiftDown();

        // Wait for the user to lower the arm.
        robotHardware.waitForArmDown();

        // Initialize the robot.
        robotHardware.initializeRobot();

        // If stop is requested...
        if(isStopRequested()) {

            // Exit the method.
            return;

        }

        // Prompt the user to press start.
        robotHardware.log("Waiting for start...");

        // Wait for the user to press start.
        waitForStart();

        // Get current and previous gamepads.
        Gamepad currentGamepad = new Gamepad();
        Gamepad previousGamepad = new Gamepad();

        // While the op mode is active...
        while (opModeIsActive()) {

            // Update the gamepads.
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            if (currentGamepad.a && !previousGamepad.a) {
                robotHardware.toggleClaw();
            }

            if (currentGamepad.x && !previousGamepad.x) {
                robotHardware.toggleWrist();
            }

            if (currentGamepad.left_bumper) {
                robotHardware.setExtensionPosition(0);
            }
            else if(currentGamepad.right_bumper) {
                robotHardware.setExtensionPosition(5);
            }

            if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                robotHardware.previousArmPosition();
            }

            if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                robotHardware.nextArmPosition();
            }

            if (currentGamepad.dpad_left) {
                robotHardware.decrementArmPosition();
            }

            if (currentGamepad.dpad_right) {
                robotHardware.incrementArmPosition();
            }

            if (currentGamepad.right_trigger > TRIGGER_THRESHOLD) {
                robotHardware.raiseLift();
            } else if (currentGamepad.left_trigger > TRIGGER_THRESHOLD) {
                robotHardware.lowerLift();
            } else {
                robotHardware.stopLift();
            }

            if(currentGamepad.y && !previousGamepad.y) {
                robotHardware.toggleTurtleMode();
            }

            // Update the robot hardware.
            robotHardware.update();

            // Update the telemetry.
            telemetry.update();

        }

    }

}