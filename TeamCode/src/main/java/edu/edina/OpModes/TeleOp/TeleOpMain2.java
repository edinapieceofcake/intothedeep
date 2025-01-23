package edu.edina.OpModes.TeleOp;

import static edu.edina.OpModes.TeleOp.TeleOpForScrimmage.MAXIMUM_SLIDE_EXTENSION_IN_SUBMERSIBLE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import edu.edina.Libraries.Robot.Drivetrain;
import edu.edina.Libraries.Robot.RobotHardware;

@Config
@TeleOp
@Disabled
public class TeleOpMain2 extends LinearOpMode {

    /*

    Robot Controls

    Normal Mode

    - left stick = move robot
    - right stick = rotate robot
    - a = toggle claw
    - x = chamber
    - y = basket
    - b = wall
    - right bumper = ground
    - left bumper = toggle swivel
    - left trigger = hold for turtle
    - back = toggle ascend

    Debug Mode (hold right trigger)

    - a = rezero arm
    - x = rezero slide
    - b = rezero lift
    - y = toggle wrist
    - robot is always fast in debug mode

    Submersible Controls
    (in normal mode, toggled with dpad up)
    (in debug mode, always enabled)

    - dpad up = increment arm
    - dpad down = decrement arm
    - dpad right = extend slide
    - dpad left = retract slide

    */

    // Trigger threshold
    private static final double TRIGGER_THRESHOLD = 0.5;

    // Ascending value
    private boolean ascending;
    private boolean turtleMode;
    private Drivetrain drivetrain;

    // Current gamepad
    private Gamepad currentGamepad1 = new Gamepad();

    // Previous gamepad
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad currentGamepad2 = new Gamepad();

    // Previous gamepad
    private Gamepad previousGamepad2 = new Gamepad();

    // Robot hardware
    private RobotHardware robotHardware;

    // Runs the op mode.
    //
    //
    //
    public void runOpMode() throws InterruptedException {

        // Get hardware.
        robotHardware = new RobotHardware(this);

        // Prompt the user to press start.
        robotHardware.log("Waiting for start...");

        // Wait for the user to press start.
        waitForStart();

        // Lower the arm.
        robotHardware.setArmGroundPosition();

        // Open the claws.
        robotHardware.openClaws();

        // Lowers the wrist.
        robotHardware.lowerWrist();

        // Resets the swivel.
        robotHardware.swivelSetHorizontal();

        // Get current and previous gamepads.
        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        // Reverse the robot.
        robotHardware.setReversed(true);

        robotHardware.initializeLights();

        // While the op mode is active...
        while (opModeIsActive()) {

            // Update the gamepads.
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            // If the right trigger is down...
            if(currentGamepad1.right_trigger > TRIGGER_THRESHOLD && !(currentGamepad2.right_trigger > TRIGGER_THRESHOLD)) {

                // Handle debug mode.
                handleDebugMode();

            }

            // Otherwise (if the right trigger is up)...
            else {

                // Handle normal mode.
                handleNormalMode();

            }
            if(currentGamepad2.right_trigger > TRIGGER_THRESHOLD && !(currentGamepad1.right_trigger > TRIGGER_THRESHOLD)) {

                // Handle debug mode.
                handleDebugMode();

            }

            // Otherwise (if the right trigger is up)...
            else {

                // Handle normal mode.
                handleNormalMode();

            }

            // Update the robot hardware.
            if (robotHardware.isArmNearSubmersiblePosition())
                robotHardware.update();
            else
                robotHardware.update();
            robotHardware.updateHardwareInteractions();

            // Update the telemetry.
            telemetry.update();

        }

    }

    // Handles debug mode.
    private void handleDebugMode() {

        // Set debugging to true.
        robotHardware.setDebugging(true);

        // Handle the submersible controls.
        handleSubmersibleControls();

        // If the user pressed y...
        if (currentGamepad1.y && !previousGamepad1.y) {

            // Toggle wrist.
            robotHardware.toggleWrist();

        }

        // Arm rezeroing
        //////////////////////////////////////////////////////////////////////

        // If the user pressed a...
        if (currentGamepad1.a && !previousGamepad1.a) {

            // Start arm rezeroing.
            robotHardware.startArmRezeroing();

        }

        // If the user released a...
        if (!currentGamepad1.a && previousGamepad1.a) {

            // Stop arm rezeroing.
            robotHardware.stopArmRezeroing();

        }

        // Slide rezeroing
        //////////////////////////////////////////////////////////////////////

        // If the user pressed x...
        if (currentGamepad1.x && !previousGamepad1.x) {

            // Start slide rezeroing.
            robotHardware.startSlideRezeroing();

        }

        // If the user released x...
        if (!currentGamepad1.x && previousGamepad1.x) {

            // Start slide rezeroing.
            robotHardware.stopSlideRezeroing();

        }

        // Lift rezeroing
        //////////////////////////////////////////////////////////////////////

        // If the user pressed b...
        if (currentGamepad1.b && !previousGamepad1.b) {

            // Start lift rezeroing.
            robotHardware.startLiftRezeroing();

        }

        // If the user released b...
        if (!currentGamepad1.b && previousGamepad1.b) {

            // Start lift rezeroing.
            robotHardware.stopLiftRezeroing();

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
        if (currentGamepad1.y && !previousGamepad1.y) {

            // Clear any pending actions.
            robotHardware.clearActions();

            // Raise the sample.
            robotHardware.raiseSample();

        }

        // Submersible
        //////////////////////////////////////////////////////////////////////

        if (robotHardware.isArmNearSubmersiblePosition()) {

            handleSubmersibleControls();

        }

        // If the user pressed dpad up...
        else if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {

            // Clear any pending actions.
            robotHardware.clearActions();

            // If the robot is in the high basket position...
            if (robotHardware.isArmInBasketPosition() && robotHardware.isLiftInBasketPosition()) {

                // Notify the user.
                robotHardware.beep();

            }

            // Otherwise (if the robot is not in the high basket position)...
            else {

                // Raise the wrist.
                robotHardware.raiseWrist();

                // Move the arm to the submersible position.
                robotHardware.setArmSubmersibleHoverPosition();

                // Move the lift to the ground position
                robotHardware.setLiftGroundPosition();

                // Use the low chamber extension.
                robotHardware.setMinimumExtension();

            }

        }

        // Ground
        //////////////////////////////////////////////////////////////////////

        // If user pressed right bumper...
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {

            // If the robot is in the basket position...
            if (robotHardware.isArmInBasketPosition() && robotHardware.isLiftInBasketPosition()) {

                // Notify the user.
                robotHardware.beep();

            }

            else {

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

        }

        // Claw
        //////////////////////////////////////////////////////////////////////

        // If the user pressed a...
        if (currentGamepad1.a && !previousGamepad1.a && !robotHardware.isArmNearSubmersiblePosition()) {

            // If the robot is in the basket position...
            if (robotHardware.isArmInBasketPosition() && robotHardware.isLiftInBasketPosition()) {

                // Score the sample.
                robotHardware.scoreSample();

            }

            // Otherwise, if the robot is in the chamber position...
            else if (robotHardware.isArmInChamberPosition() && robotHardware.isLiftInGroundPosition()) {

                // Score the specimen.
                robotHardware.scoreSpecimen();

            }

            // Otherwise...
            else {

                // Toggle the claw.
                //robotHardware.toggleClaw();

            }

        }
        if (currentGamepad2.a && previousGamepad2.a && robotHardware.isArmNearSubmersiblePosition()){
            //robotHardware.toggleClaw();
        }

        // High chamber
        //////////////////////////////////////////////////////////////////////

        // If the user pressed x...
        if (currentGamepad1.x && !previousGamepad1.x) {

            // Clear any pending actions.
            robotHardware.clearActions();

            // If the arm is in the submersible...
            if (robotHardware.isArmInSubmersibleHoverPosition()) {

                // Notify the user.
                robotHardware.beep();

            }

            // Otherwise (if the arm is not in the submersible)...
            else {

                // Set the wrist to high chamber hold position.
                robotHardware.setWristChamberPosition();

                // Set the swivel to clip position.
                robotHardware.swivelSetClip();

                // Move the arm to the high chamber position.
                robotHardware.setArmChamberPosition();

                // Move the lift to the ground position.
                robotHardware.setLiftGroundPosition();

                // Use the high chamber extension.
                robotHardware.setMinimumExtension();

            }

        }

        // Wall
        //////////////////////////////////////////////////////////////////////

        // If user pressed b...
        if (currentGamepad1.b && !previousGamepad1.b) {

            // Clear any pending actions.
            robotHardware.clearActions();

            // Set the wrist to wall position.
            robotHardware.setWristWallPosition();

            // Resets the swivel.
            robotHardware.swivelSetHorizontal();

            // Move the arm to the wall position.
            robotHardware.setArmWallPosition(true);

            // Move the lift to the ground position
            robotHardware.setLiftGroundPosition();

            // Fully retract the slide.
            robotHardware.setMinimumExtension();

        }

        // Ascend
        //////////////////////////////////////////////////////////////////////

        // If the user pressed back...
        if (currentGamepad1.back && !previousGamepad1.back) {

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

        robotHardware.setTurtleMode(currentGamepad1.left_trigger > TRIGGER_THRESHOLD || robotHardware.isArmNearSubmersiblePosition());
        /*Potential Addition!!!: Toggle turtle mode for player 2*/

        // Toggle swivel
        //////////////////////////////////////////////////////////////////////

        // If the user tapped left bumper...
        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper && !robotHardware.isArmNearSubmersiblePosition()) {

            // Toggle the swivel.
            robotHardware.toggleSwivel();

        }
        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper && robotHardware.isArmNearSubmersiblePosition()) {

            // Toggle the swivel.
            robotHardware.toggleSwivel();

        }

    }

    private void handleSubmersibleControls() {

        // Decrement arm
        //////////////////////////////////////////////////////////////////////

        // If the user tapped dpad up...
        if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {

            // Decrement the arm position.
            robotHardware.decrementArmPosition();

        }

        // Increment arm
        //////////////////////////////////////////////////////////////////////

        // If the user tapped dpad down...
        if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {

            // Increment the arm position.
            robotHardware.incrementArmPosition();

        }

        // Extend slide
        //////////////////////////////////////////////////////////////////////

        // If the user tapped dpad right...
        if (currentGamepad2.dpad_right) {

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
        if (currentGamepad2.dpad_left) {

            // Retract the slide.
            robotHardware.retractSlide();

        }

    }

}