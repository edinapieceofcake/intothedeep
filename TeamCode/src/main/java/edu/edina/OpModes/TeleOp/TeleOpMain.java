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
    - left bumper = hold for turtle
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

        robotHardware.setReversed(true);

        // While the op mode is active...
        while (opModeIsActive()) {

            // Update the gamepads.
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            // If the user pressed y...
            if (currentGamepad.y && !previousGamepad.y) {

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

                // Determine whether the arm is in the high basket position.
                boolean isArmInHighBasketPosition = robotHardware.isArmInHighBasketPosition();

                // Determine whether the arm is in the high chamber position.
                boolean isArmInHighChamberPosition = robotHardware.isArmInHighChamberPosition();

                // Determine whether to disallow the submersible preset (so the robot stays within the expansion box).
                boolean disallowSubmersiblePreset = isArmInHighBasketPosition || isArmInHighChamberPosition;

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

            // If user pressed right bumper...
            if (currentGamepad.right_bumper && !previousGamepad.right_bumper) {

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

            if (currentGamepad.back && !previousGamepad.back) {
                robotHardware.setArmAlmostGroundPosition();
                if (ascending) {
                    robotHardware.setLiftGroundPosition();
                }
                else {
                    robotHardware.setLiftHighBasketPosition();
                }
                ascending = !ascending;
            }

            // Set turtle mode.
            robotHardware.setTurtleMode(currentGamepad.left_bumper);

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