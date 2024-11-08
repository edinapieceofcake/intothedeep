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

            // If the user pressed a...
            if (currentGamepad.a && !previousGamepad.a) {

                // Toggle the claw.
                robotHardware.toggleClaw();

            }

            // If the user pressed x...
            if (currentGamepad.x && !previousGamepad.x) {

                // Toggle the wrist.
                robotHardware.toggleWrist();

            }

            // If the user pressed left bumper...
            if (currentGamepad.left_bumper) {

                // Retract the extension.
                robotHardware.setExtensionPosition(0);

            }

            // Otherwise, if the user pressed right bumper...
            else if(currentGamepad.right_bumper) {

                // Extend the extension.
                robotHardware.setExtensionPosition(5);

            }

            // If the user pressed dpad down...
            if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {

                // Go to the previous arm position.
                robotHardware.previousArmPosition();

            }

            // If the user pressed dpad up...
            if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {

                // Go to the next arm position.
                robotHardware.nextArmPosition();

            }

            // If the user pressed dpad left...
            if (currentGamepad.dpad_left) {

                // Decrement the arm position.
                robotHardware.decrementArmPosition();

            }

            // If the user pressed dpad right...
            if (currentGamepad.dpad_right) {

                // Increment the arm position.
                robotHardware.incrementArmPosition();

            }

            // If the user is holding right trigger...
            if (currentGamepad.right_trigger > TRIGGER_THRESHOLD) {

                // If the arm is not nearly down...
                if (!robotHardware.isArmNearlyDown()) {

                    // Raise the lift.
                    robotHardware.raiseLift();

                }

            }

            // Otherwise, if the user is holding left trigger...
            else if (currentGamepad.left_trigger > TRIGGER_THRESHOLD) {

                // Lower the lift.
                robotHardware.lowerLift();

            }

            // If the user pressed y...
            if(currentGamepad.y && !previousGamepad.y) {

                // Toggle turtle mode.
                robotHardware.toggleTurtleMode();
                
            }

            // Update the robot hardware.
            robotHardware.update();

            // Update the telemetry.
            telemetry.update();

        }

    }

}