package edu.edina.OpModes.TeleOp;

import static edu.edina.OpModes.TeleOp.TeleOpForScrimmage.MAXIMUM_SLIDE_EXTENSION_IN_SUBMERSIBLE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import edu.edina.Libraries.Robot.RobotHardware;

@Config
@TeleOp
public class TeleOpMain extends LinearOpMode {

    /*

    Robot Controls

    - left stick = move robot
    - right stick = rotate robot
    - a = toggle claw
    - x = chamber
    - y = basket
    - b = submersible
    - right bumper = ground
    - left trigger = hold for turtle
    - back = toggle ascend
    - dpad up = increment arm
    - dpad down = decrement arm
    - dpad right = extend slide
    - dpad left = retract slide

    */

    // Trigger threshold
    private static final double TRIGGER_THRESHOLD = 0.5;

    private boolean ascending;

    // Runs the op mode.
    public void runOpMode() throws InterruptedException {

        // Get hardware.
        RobotHardware robotHardware = new RobotHardware(this);

        // Prompt the user to press start.
        robotHardware.log("Waiting for start...");

        // Wait for the user to press start.
        waitForStart();

        // Open the claw.
        robotHardware.openClaw();

        // Lowers the wrist.
        robotHardware.lowerWrist();

        // Get current and previous gamepads.
        Gamepad currentGamepad = new Gamepad();
        Gamepad previousGamepad = new Gamepad();

        // While the op mode is active...
        while (opModeIsActive()) {

            // Update the gamepads.
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            // Reverse the robot if appropriate.
            robotHardware.setReversed(!robotHardware.isArmAlmostGroundOrLower());

            // If the user pressed y...
            if (currentGamepad.y && !previousGamepad.y) {

                // Clear any pending actions.
                robotHardware.clearActions();

                // Raise the wrist.
                robotHardware.raiseWrist();

                // Move the arm to the high basket position.
                robotHardware.setArmHighBasketPosition();

                // Move the lift to the high basket position
                robotHardware.setLiftHighBasketPosition();

                // Use the high basket extension.
                robotHardware.setHighBasketExtension();

            }

            // If the user pressed b...
            if (currentGamepad.b && !previousGamepad.b) {

                // Clear any pending actions.
                robotHardware.clearActions();

                // If the robot is in the high basket position...
                if (robotHardware.isArmInHighBasketPosition() && robotHardware.isLiftInHighBasketPosition()) {

                    // Notify the user.
                    robotHardware.beep();

                }

                // Otherwise (if the robot is not in the high basket position)...
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

            // If user pressed right bumper...
            if (currentGamepad.right_bumper && !previousGamepad.right_bumper) {

                // Clear any pending actions.
                robotHardware.clearActions();

                // Raise the wrist.
                robotHardware.raiseWrist();

                // Move the arm to the ground position.
                robotHardware.setArmGroundPosition();

                // Move the lift to the ground position
                robotHardware.setLiftGroundPosition();

                // Fully retract the slide.
                robotHardware.setMinimumExtension();

            }

            // If the user pressed a...
            if (currentGamepad.a && !previousGamepad.a) {

                // If the robot is in the basket position...
                if (robotHardware.isArmInHighBasketPosition() && robotHardware.isLiftInHighBasketPosition()) {

                    // Score the sample.
                    robotHardware.scoreSample();

                }

                // Otherwise, if the robot is in the chamber position...
                else if (robotHardware.isArmInHighChamberPosition() && robotHardware.isLiftInGroundPosition()) {

                    // Score the specimen.
                    robotHardware.scoreSpecimen();

                }

                // Otherwise...
                else {

                    // Toggle the claw.
                    robotHardware.toggleClaw();

                }

            }

            // If the user pressed x...
            if (currentGamepad.x && !previousGamepad.x) {

                // Clear any pending actions.
                robotHardware.clearActions();

                // If the arm is in the submersible...
                if (robotHardware.isArmInSubmersiblePosition()) {

                    // Notify the user.
                    robotHardware.beep();

                }

                // Otherwise (if the arm is not in the submersible)...
                else {

                    // Lower the wrist.
                    robotHardware.lowerWrist();

                    // Move the arm to the high chamber position.
                    robotHardware.setArmHighChamberPosition();

                    // Move the lift to the ground position.
                    robotHardware.setLiftGroundPosition();

                    // Use the high chamber extension.
                    robotHardware.setMinimumExtension();

                }

            }

            // If the user pressed back...
            if (currentGamepad.back && !previousGamepad.back) {

                // Clear any pending actions.
                robotHardware.clearActions();

                // Move the arm to the ascent position.
                robotHardware.setArmAscentPosition();

                // If we are ascending...
                if (ascending) {

                    // Descend.
                    robotHardware.setLiftAscendDownPosition();

                }

                // Otherwise (if we are descending)...
                else {

                    // Ascend.
                    robotHardware.setLiftAscendPosition();

                }

                // Toggle the ascending value.
                ascending = !ascending;

            }

            // Set turtle mode.
            robotHardware.setTurtleMode(currentGamepad.left_trigger > TRIGGER_THRESHOLD);

            // If the user tapped dpad up...
            if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {

                // Decrement the arm position.
                robotHardware.decrementArmPosition();

            }

            // If the user tapped dpad down...
            if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {

                // Increment the arm position.
                robotHardware.incrementArmPosition();

            }

            // If the user tapped dpad right...
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

            // If the user tapped dpad left...
            if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {

                // Retract the slide.
                robotHardware.retractSlide();

            }

            // Update the robot hardware.
            robotHardware.update();
            robotHardware.updateHardwareInteractions();

            // Update the telemetry.
            telemetry.update();
        }
    }

}