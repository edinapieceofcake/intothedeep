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

    Normal Mode
    - left stick = move robot
    - right stick = rotate robot
    - a = toggle claw
    - x = toggle wrist
    - y = toggle turtle mode
    - dpad up = next arm position
    - dpad down = previous arm position
    - dpad right = increment arm position
    - dpad left = decrement arm position

    Preset Mode (hold right trigger)
    - a = ground
    - x = low basket
    - y = high basket

    Manual Mode (hold left trigger)
    - dpad up = raise lift
    - dpad down = lower lift
    - dpad right = extend slide
    - dpad left = retract slide

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

            // If left trigger is down...
            if(currentGamepad.left_trigger > TRIGGER_THRESHOLD) {

                // Manual mode
                //////////////////////////////////////////////////////////////////////

                // If the user is holding dpad up...
                if (currentGamepad.dpad_up) {

                    // If the arm is not nearly down...
                    if (!robotHardware.isArmNearlyDown()) {

                        // Raise the lift.
                        robotHardware.raiseLift();

                    }

                }

                // If the user is holding dpad down...
                if (currentGamepad.dpad_down) {

                    // Lower the lift.
                    robotHardware.lowerLift();

                }

                // If the user is holding dpad right...
                if (currentGamepad.dpad_right) {

                    // Extend the extension.
                    robotHardware.extendExtension();

                }

                // If the user is holding dpad left...
                if(currentGamepad.dpad_left) {

                    // Retract the extension.
                    robotHardware.retractExtension();

                }

            }

            // Otherwise, if right trigger is down...
            else if(currentGamepad.right_trigger > TRIGGER_THRESHOLD) {

                // Preset mode
                //////////////////////////////////////////////////////////////////////

                // If the user pressed a...
                if(currentGamepad.a && !previousGamepad.a) {

                    // Raise the wrist.
                    robotHardware.raiseWrist();

                    // Move the arm to the ground position.
                    robotHardware.setArmGroundPosition();

                    // Move the lift to the ground position
                    robotHardware.setLiftGroundPosition();

                    // Fully retract the slide.
                    robotHardware.setMinimumExtension();

                }

                // If the user pressed x...
                if(currentGamepad.x && !previousGamepad.x) {

                    // Raise the wrist.
                    robotHardware.raiseWrist();

                    // Move the arm to the low basket position.
                    robotHardware.setArmLowBasketPosition();

                    // Move the lift to the ground position
                    robotHardware.setLiftGroundPosition();

                    // Use the low basket extension.
                    robotHardware.setLowBasketExtension();

                }

                // If the user pressed y...
                if(currentGamepad.y && !previousGamepad.y) {

                    // Raise the wrist.
                    robotHardware.raiseWrist();

                    // Move the arm to the high basket position.
                    robotHardware.setArmHighBasketPosition();

                    // Move the lift to the high basket position
                    robotHardware.setLiftHighBasketPosition();

                    // Fully retract the slide.
                    robotHardware.setMinimumExtension();

                }

            }

            // Otherwise (if both triggers are up)...
            else {

                // Normal mode
                //////////////////////////////////////////////////////////////////////

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

                // If the user pressed y...
                if(currentGamepad.y && !previousGamepad.y) {

                    // Toggle turtle mode.
                    robotHardware.toggleTurtleMode();

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

            }

            // Update the robot hardware.
            robotHardware.update();

            // Update the telemetry.
            telemetry.update();

        }

    }

}