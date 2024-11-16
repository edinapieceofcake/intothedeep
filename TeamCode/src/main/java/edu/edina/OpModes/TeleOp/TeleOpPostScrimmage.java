package edu.edina.OpModes.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import edu.edina.Libraries.Robot.MiniAutoMode;
import edu.edina.Libraries.Robot.RobotHardware;

@Config
@TeleOp
public class TeleOpPostScrimmage extends LinearOpMode {

    /*

    Robot Controls

    Normal Mode
    - left stick = move robot
    - right stick = rotate robot
    - a = toggle claw
    - y = toggle turtle mode
    - x = wrist toggle
    - right bumper = raise arm
    - left bumper = lower arm

    Sample basket scoring (hold left trigger)
    - x = low basket
    - y = high basket
    - a = ground

    Submersible & rungs (hold right trigger) (driving reversed)
    - dpad-up = high rung
    - dpad-down = low rung
    - dpad-right = submersible
    - a = toggle claw
    */

    // Trigger threshold
    private static final double TRIGGER_THRESHOLD = 0.5;

    // Runs the op mode.
    public void runOpMode() throws InterruptedException {

        // Get hardware.
        RobotHardware robotHardware = new RobotHardware(this);

        /*// Wait for the user to lower the lift.
        robotHardware.waitForLiftDown();

        // Wait for the user to lower the arm.
        robotHardware.waitForArmDown();

        // Initialize the robot.
        robotHardware.initializeRobot();*/

        // If stop is requested...
        if (isStopRequested()) {

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

            boolean subAndRungs = currentGamepad.right_trigger > TRIGGER_THRESHOLD;

            robotHardware.setReversed(subAndRungs);

            // If left trigger is down...
            if (subAndRungs) {

                // Specimen and Submersible
                //////////////////////////////////////////////////////////////////////

                // If the user pressed dpad up...
                if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                    robotHardware.setTurtleMode(true);

                    // Move the arm to the high chamber position.
                    robotHardware.setArmHighChamberPosition();

                    // Move the lift to the ground position
                    robotHardware.setLiftGroundPosition();

                    // Use the high chamber extension.
                    robotHardware.setMinimumExtension();

                    robotHardware.lowerWrist();
                }

                // If the user pressed dpad down...
                if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                    robotHardware.setTurtleMode(true);

                    // Move the arm to the low chamber position.
                    robotHardware.setArmLowChamberPosition();

                    // Move the lift to the ground position
                    robotHardware.setLiftGroundPosition();

                    // Use the low chamber extension.
                    robotHardware.setMinimumExtension();

                    robotHardware.lowerWrist();
                }

                // If the user pressed dpad right...
                if (currentGamepad.dpad_right && !previousGamepad.dpad_right) {
                    robotHardware.setTurtleMode(true);

                    // Move the arm to the submersible position.
                    robotHardware.setArmSubmersiblePosition();

                    // Move the lift to the ground position
                    robotHardware.setLiftGroundPosition();

                    // Use the low chamber extension.
                    robotHardware.setMinimumExtension();

                }

                if (currentGamepad.a) {
                    robotHardware.toggleClaw();
                }

                if (currentGamepad.x) {
                    specimenScoring(robotHardware);
                }
            }

            // Otherwise, if right trigger is down...
            else if (currentGamepad.left_trigger > TRIGGER_THRESHOLD) {

                // Sample mode
                //////////////////////////////////////////////////////////////////////

                // If the user pressed a...
                if (currentGamepad.a && !previousGamepad.a) {

                    // Raise the wrist.
                    //robotHardware.raiseWrist();

                    robotHardware.setTurtleMode(false);

                    // Move the arm to the ground position.
                    robotHardware.setArmGroundPosition();

                    // Move the lift to the ground position
                    robotHardware.setLiftGroundPosition();

                    // Fully retract the slide.
                    robotHardware.setMinimumExtension();

                }

                // If the user pressed b...
                if (currentGamepad.b && !previousGamepad.b) {

                    // Raise the wrist.
                    //robotHardware.raiseWrist();

                    robotHardware.setTurtleMode(false);

                    // Move the arm to the ground position.
                    robotHardware.setArmAlmostGroundPosition();

                    // Move the lift to the ground position
                    robotHardware.setLiftGroundPosition();

                    // Fully retract the slide.
                    robotHardware.setMinimumExtension();

                }

                // low basket
                if (currentGamepad.x && !previousGamepad.x) {
                    // Raise the wrist.
                    //robotHardware.raiseWrist();

                    robotHardware.setTurtleMode(true);

                    // Move the arm to the low basket position.
                    robotHardware.setArmLowBasketPosition();

                    // Move the lift to the ground position
                    robotHardware.setLiftGroundPosition();

                    // Use the low basket extension.
                    robotHardware.setLowBasketExtension();

                }

                // If the user pressed y...
                if (currentGamepad.y && !previousGamepad.y) {

                    // Raise the wrist.
                    //robotHardware.raiseWrist();

                    robotHardware.setTurtleMode(true);

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

                // Universal mode
                //////////////////////////////////////////////////////////////////////

                // If the user pressed a...
                if (currentGamepad.a && !previousGamepad.a) {

                    // Toggle the claw.
                    robotHardware.toggleClaw();

                }

                // If the user pressed y...
                if (currentGamepad.y && !previousGamepad.y) {

                    // Toggle turtle mode.
                    robotHardware.toggleTurtleMode();

                }

                /*
                // If the user pressed dpad left...
                if (currentGamepad.dpad_left) {

                    // Go to the previous arm position.
                    robotHardware.previousArmPosition();

                }

                // If the user pressed dpad right...
                if (currentGamepad.dpad_right) {

                    // Go to the next arm position.
                    robotHardware.nextArmPosition();

                }
                 */

            }

            if (currentGamepad.x && !previousGamepad.x) {

                robotHardware.toggleWrist();

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
            robotHardware.updateHardwareInteractions();

            // Update the telemetry.
            telemetry.update();
        }
    }

    public void specimenScoring(RobotHardware hw) {
        hw.startMiniAutoMode();

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > TRIGGER_THRESHOLD && gamepad1.x) {
                boolean stillScoring = hw.update(MiniAutoMode.SCORE);
                if (!stillScoring) {
                    hw.raiseWrist();
                    break;
                }
            } else {
                break;
            }
        }
    }
}