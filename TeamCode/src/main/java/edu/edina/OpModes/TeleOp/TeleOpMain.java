package edu.edina.OpModes.TeleOp;

import static edu.edina.OpModes.TeleOp.TeleOpForScrimmage.MAXIMUM_SLIDE_EXTENSION_IN_SUBMERSIBLE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import edu.edina.Libraries.Robot.Arm;
import edu.edina.Libraries.Robot.MoveArm;
import edu.edina.Libraries.Robot.RobotHardware;

@Config
@TeleOp
public class TeleOpMain extends LinearOpMode {

    /*

    Robot Controls

    Normal Mode

    - left stick = move robot
    - right stick button = toggle swivel
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

    Debug Mode (hold right trigger)

    - y = wall
    - a = rezero arm
    - x = rezero slide
    - b = rezero lift
    - the robot is never forced into turtle mode

    */

    // Trigger threshold
    private static final double TRIGGER_THRESHOLD = 0.5;

    // Ascending value
    private boolean ascending;

    // Current gamepad
    private Gamepad currentGamepad = new Gamepad();

    // Previous gamepad
    private Gamepad previousGamepad = new Gamepad();

    // Robot hardware
    private RobotHardware robotHardware;

    // Runs the op mode.
    public void runOpMode() throws InterruptedException {

        // Get hardware.
        robotHardware = new RobotHardware(this);

        // Prompt the user to press start.
        robotHardware.log("Waiting for start...");

        // Wait for the user to press start.
        waitForStart();

        // Lower the arm.
        robotHardware.setArmGroundPosition();

        // Open the claw.
        robotHardware.openClaw();

        // Lowers the wrist.
        robotHardware.lowerWrist();

        // Resets the swivel.
        robotHardware.swivelSetHorizontal();

        // Get current and previous gamepads.
        currentGamepad = new Gamepad();
        previousGamepad = new Gamepad();

        // While the op mode is active...
        while (opModeIsActive()) {

            // Update the gamepads.
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            // Reverse the robot if appropriate.
            robotHardware.setReversed(!robotHardware.isArmInGroundPosition());

            // If the right trigger is down...
            if(currentGamepad.right_trigger > TRIGGER_THRESHOLD) {

                // Handle debug mode.
                handleDebugMode();

            }

            // Otherwise (if the right trigger is up)...
            else {

                // Handle normal mode.
                handleNormalMode();

            }

            // Update the robot hardware.
            robotHardware.update();
            robotHardware.updateHardwareInteractions();

            // Update the telemetry.
            telemetry.update();

        }

    }

    // Handles debug mode.
    private void handleDebugMode() {

        // Set debugging to true
        robotHardware.setDebugging(true);

        // Arm rezeroing
        //////////////////////////////////////////////////////////////////////

        // If the user pressed a...
        if (currentGamepad.a && !previousGamepad.a) {

            // Start arm rezeroing.
            robotHardware.startArmRezeroing();

        }

        // If the user released a...
        if (!currentGamepad.a && previousGamepad.a) {

            // Stop arm rezeroing.
            robotHardware.stopArmRezeroing();

        }

        // Slide rezeroing
        //////////////////////////////////////////////////////////////////////

        // If the user pressed x...
        if (currentGamepad.x && !previousGamepad.x) {

            // Start slide rezeroing.
            robotHardware.startSlideRezeroing();

        }

        // If the user released x...
        if (!currentGamepad.x && previousGamepad.x) {

            // Start slide rezeroing.
            robotHardware.stopSlideRezeroing();

        }

        // Lift rezeroing
        //////////////////////////////////////////////////////////////////////

        // If the user pressed b...
        if (currentGamepad.b && !previousGamepad.b) {

            // Start lift rezeroing.
            robotHardware.startLiftRezeroing();

        }

        // If the user released b...
        if (!currentGamepad.b && previousGamepad.b) {

            // Start lift rezeroing.
            robotHardware.stopLiftRezeroing();

        }

        // If the user pressed y...
        if (currentGamepad.y && !previousGamepad.y) {

            // Clear any pending actions.
            robotHardware.clearActions();

            // Set the wrist to wall position.
            robotHardware.setWristWallPosition();

            // Resets the swivel.
            robotHardware.swivelSetHorizontal();

            // Move the arm to the wall position.
            robotHardware.setArmWallPosition();

            // Move the lift to the ground position
            robotHardware.setLiftGroundPosition();

            // Fully retract the slide.
            robotHardware.setMinimumExtension();

        }

    }

    // Handles normal mode.
    private void handleNormalMode() {

        // Set debugging to false
        robotHardware.setDebugging(false);

        // Stop rezeroing.
        //////////////////////////////////////////////////////////////////////

        // Stop rezeroing.
        robotHardware.stopArmRezeroing();
        robotHardware.stopLiftRezeroing();
        robotHardware.stopSlideRezeroing();

        // High basket
        //////////////////////////////////////////////////////////////////////

        // If the user pressed y...
        if (currentGamepad.y && !previousGamepad.y) {

            // Clear any pending actions.
            robotHardware.clearActions();

            // Raise the sample.
            robotHardware.raiseSample();

        }

        // Submersible
        //////////////////////////////////////////////////////////////////////

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

        // Ground
        //////////////////////////////////////////////////////////////////////

        // If user pressed right bumper...
        if (currentGamepad.right_bumper && !previousGamepad.right_bumper) {

            // Clear any pending actions.
            robotHardware.clearActions();

            // Raise the wrist.
            robotHardware.raiseWrist();

            // Resets the swivel.
            robotHardware.swivelSetHorizontal();

            // Move the arm to the ground position.
            robotHardware.setArmGroundPosition();

            // Move the lift to the ground position
            robotHardware.setLiftGroundPosition();

            // Fully retract the slide.
            robotHardware.setMinimumExtension();

        }

        // Claw
        //////////////////////////////////////////////////////////////////////

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

        // High chamber
        //////////////////////////////////////////////////////////////////////

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

                // Set the wrist to high chamber hold position.
                robotHardware.setWristHighChamberHoldPosition();

                // Set the swivel to clip position.
                robotHardware.swivelSetClip();

                // Move the arm to the high chamber position.
                robotHardware.setArmHighChamberPosition();

                // Move the lift to the ground position.
                robotHardware.setLiftGroundPosition();

                // Use the high chamber extension.
                robotHardware.setMinimumExtension();

            }

        }

        // Ascend
        //////////////////////////////////////////////////////////////////////

        // If the user pressed back...
        if (currentGamepad.back && !previousGamepad.back) {

            // Clear any pending actions.
            robotHardware.clearActions();

            // If we are ascending...
            if (ascending) {

                // Ascend.
                robotHardware.ascend();

            }

            // Otherwise (if we are not ascending)...
            else {

                // Descend.
                robotHardware.descend();

            }

            // Toggle the ascending value.
            ascending = !ascending;

        }

        // Turtle mode
        //////////////////////////////////////////////////////////////////////

        // Set turtle mode.
        robotHardware.setTurtleMode(currentGamepad.left_trigger > TRIGGER_THRESHOLD);

        // Decrement arm
        //////////////////////////////////////////////////////////////////////

        // If the user tapped dpad up...
        if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {

            // Decrement the arm position.
            robotHardware.decrementArmPosition();

        }

        // Increment arm
        //////////////////////////////////////////////////////////////////////

        // If the user tapped dpad down...
        if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {

            // Increment the arm position.
            robotHardware.incrementArmPosition();

        }

        // Extend slide
        //////////////////////////////////////////////////////////////////////

        // If the user tapped dpad right...
        if (currentGamepad.dpad_right && !previousGamepad.dpad_right) {

            // Determine whether the arm is nearly down.
            boolean isArmNearlyDown = robotHardware.isArmNearlyDown();

            // Determine whether the arm is in the submersible position.
            boolean isArmNearSubmersiblePosition = robotHardware.isArmNearSubmersiblePosition();

            // Get the current slide extension.
            double currentSlideExtension = robotHardware.getCurrentSlideExtension();

            // Determine whether the slide is maximally extended in the submersible.
            boolean isSlideMaximallyExtendedInSubmersible = isArmNearSubmersiblePosition && currentSlideExtension >= MAXIMUM_SLIDE_EXTENSION_IN_SUBMERSIBLE;

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

        // Retract slide
        //////////////////////////////////////////////////////////////////////

        // If the user tapped dpad left...
        if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {

            // Retract the slide.
            robotHardware.retractSlide();

        }

        // Toggle swivel
        //////////////////////////////////////////////////////////////////////

        // If the user tapped right stick...
        if (currentGamepad.right_stick_button && !previousGamepad.right_stick_button) {

            // Toggle the swivel.
            robotHardware.toggleSwivel();

        }

    }

}