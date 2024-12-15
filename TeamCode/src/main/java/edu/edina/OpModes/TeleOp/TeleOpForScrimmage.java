package edu.edina.OpModes.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import edu.edina.Libraries.Robot.RobotHardware;

@Config
@TeleOp
@Disabled
public class TeleOpForScrimmage extends LinearOpMode {

    /*

    Robot Controls

    Normal Mode
    - left stick = move robot
    - right stick = rotate robot
    - a = toggle claw
    - y = toggle turtle mode
    - right bumper = raise arm
    - left bumper = lower arm

    Preset Mode (hold right trigger)
    - a = ground
    - x = low basket
    - y = high basket
    - d-pad up = high chamber
    - d-pad down = low chamber
    - d-pad right = submersible

    Manual Mode (hold left trigger)
    - dpad up = raise lift
    - dpad down = lower lift
    - dpad right = extend slide
    - dpad left = retract slide

    */

    // Maximum slide extension in submersible (so the robot stays within the expansion box)
    public static double MAXIMUM_SLIDE_EXTENSION_IN_SUBMERSIBLE = 7;

    // Trigger threshold
    public static double TRIGGER_THRESHOLD = 0.5;

    // Current gamepad
    private Gamepad currentGamepad = new Gamepad();

    // Previous gamepad
    private Gamepad previousGamepad = new Gamepad();

    // Runs the op mode.
    public void runOpMode() throws InterruptedException {

        // Get hardware.
        RobotHardware robotHardware = new RobotHardware(this);

        // Prompt the user to press start.
        robotHardware.log("Waiting for start...");

        // Wait for the user to press start.
        waitForStart();

        // Move the arm to the ground position.
        robotHardware.setArmGroundPosition();

        // Open the claw.
        robotHardware.openClaw();

        // Lowers the wrist.
        robotHardware.lowerWrist();

        // While the op mode is active...
        while (opModeIsActive()) {

            // Update the gamepads.
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            // If left trigger is down...
            if(currentGamepad.left_trigger > TRIGGER_THRESHOLD) {

                // Manual mode
                //////////////////////////////////////////////////////////////////////

                // If the user pressed dpad right...
                if (currentGamepad.dpad_right && !previousGamepad.dpad_right) {

                    // Determine whether the arm is nearly down.
                    boolean isArmNearlyDown = robotHardware.isArmNearlyDown();

                    // Determine whether the arm is in the submersible position.
                    boolean isArmInSubmersiblePosition = robotHardware.isArmInSubmersiblePosition();

                    // Get the current slide extension.
                    double currentSlideExtension = robotHardware.getCurrentSlideExtension();

                    // Determine whether the slide is maximally extended in the submersible.
                    boolean isSlideMaximallyExtendedInSubmersible = isArmInSubmersiblePosition && currentSlideExtension >= MAXIMUM_SLIDE_EXTENSION_IN_SUBMERSIBLE;

                    // Determine whether to disallow slide extension (so the robot stays within the expansion box).
                    boolean disallowSlideExtension = isArmNearlyDown || isSlideMaximallyExtendedInSubmersible;

                    // If slide extension is disallowed...
                    if (disallowSlideExtension) {

                        // Notify the user.
                        robotHardware.beep();

                    }

                    // Otherwise (if slide extension is allowed)...
                    else {

                        // Extend the slide.
                        robotHardware.extendSlide();

                    }

                }

                // If the user pressed dpad left...
                if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {

                    // Retract the slide.
                    robotHardware.retractSlide();

                }

                // If the user is holding dpad right...
                if (currentGamepad.dpad_up) {

                    // Raise the lift.
                    robotHardware.raiseLift();

                }

                // If the user is holding dpad left...
                if(currentGamepad.dpad_down) {

                    // Lower the lift.
                    robotHardware.lowerLift();

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

                // If the user pressed dpad up...
                if(currentGamepad.dpad_up && !previousGamepad.dpad_up) {

                    // Lower the wrist.
                    robotHardware.lowerWrist();

                    // Move the arm to the high chamber position.
                    robotHardware.setArmHighChamberPosition();

                    // Move the lift to the ground position
                    robotHardware.setLiftGroundPosition();

                    // Use the high chamber extension.
                    robotHardware.setMinimumExtension();

                }

                // If the user pressed dpad down...
                if(currentGamepad.dpad_down && !previousGamepad.dpad_down) {

                    // Lower the wrist.
                    robotHardware.lowerWrist();

                    // Move the arm to the low chamber position.
                    robotHardware.setArmLowChamberPosition();

                    // Move the lift to the ground position
                    robotHardware.setLiftGroundPosition();

                    // Use the low chamber extension.
                    robotHardware.setMinimumExtension();

                }

                // If the user pressed dpad down...
                if(currentGamepad.dpad_right && !previousGamepad.dpad_right) {

                    // Determine whether the arm is in the low basket position.
                    boolean isArmInLowBasketPosition = robotHardware.isArmInLowBasketPosition();

                    // Determine whether the arm is in the high basket position.
                    boolean isArmInHighBasketPosition = robotHardware.isArmInHighBasketPosition();

                    // Determine whether to disallow the submersible preset (so the robot stays within the expansion box).
                    boolean disallowSubmersiblePreset = isArmInLowBasketPosition || isArmInHighBasketPosition;

                    // If the submersible preset is disallowed...
                    if (disallowSubmersiblePreset) {

                        // Notify the user.
                        robotHardware.beep();

                    }

                    // Otherwise (if the submersible preset is allowed)...
                    else {

                        // Raise the wrist.
                        robotHardware.raiseWrist();

                        // Move the arm to the submersible position.
                        robotHardware.setArmSubmersiblePosition();

                        // Move the lift to the ground position
                        robotHardware.setLiftGroundPosition();

                        // Use the low chamber extension.
                        robotHardware.setMinimumExtension();

                    }

                }

                // If the user pressed y...
                if(currentGamepad.y && !previousGamepad.y) {

                    // Raise the wrist.
                    robotHardware.raiseWrist();

                    // Move the arm to the high basket position.
                    robotHardware.setArmHighBasketPosition();

                    // Move the lift to the high basket position
                    robotHardware.setLiftHighBasketPosition();

                    // Use the high basket extension.
                    robotHardware.setHighBasketExtension();

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

            }

            // If the user tapped right bumper...
            if (currentGamepad.right_bumper && !previousGamepad.right_bumper) {

                // Increment the arm position.
                robotHardware.incrementArmPosition();

            }

            // If the user tapped left bumper...
            if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {

                // Increment the arm position.
                robotHardware.decrementArmPosition();

            }

            // Update the robot hardware.
            robotHardware.update();

            // Update the telemetry.
            telemetry.update();

        }

    }

}