package edu.edina.OpModes.TeleOp;

import static edu.edina.OpModes.Autonomous.AutoSample.lastPose;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import edu.edina.Libraries.Actions.Condition;
import edu.edina.Libraries.Actions.Conditions;
import edu.edina.Libraries.Actions.KillSwitchAction;
import edu.edina.Libraries.Actions.RaiseLift;
import edu.edina.Libraries.Actions.SpecimenPark;
import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.Robot.Arm;
import edu.edina.Libraries.Robot.MoveArm;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.WaitForHardware;
import edu.edina.Libraries.Robot.WaitForSlide;
import edu.edina.Libraries.Robot.WaitForTime;

@Config
@TeleOp
public class TeleOpMain extends LinearOpMode {

    /*

    Robot Controls

    Normal Mode

    - left stick = move robot
    - right stick = rotate robot
    - a = toggle claw
    - x = chamber
    - y = basket
    - b = wall
    - right bumper = submersible
    - left bumper = toggle swivel
    - left trigger = hold for turtle
    - back = toggle ascend (unimplemented)
    - dpad left = retract slide
    - dpad right = extend slide

    Debug Mode (hold right trigger)

    - a = rezero arm (unimplemented)
    - x = rezero slide (unimplemented)
    - b = rezero lift (unimplemented)

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
    private MecanumDrive drive;
    private Condition autoCondition;


    // Runs the op mode.
    public void runOpMode() throws InterruptedException {

        // Get hardware.
        robotHardware = new RobotHardware(this);
        drive = new MecanumDrive(hardwareMap, lastPose);

        // Prompt the user to press start.
        robotHardware.log("Waiting for start...");

        // Wait for the user to press start.
        waitForStart();
        autoCondition = null;

        // Lower the arm.
        robotHardware.setArmGroundPosition();

        // Open the claws.
        robotHardware.openClaws();

        // Resets the swivel.
        robotHardware.swivelSetHorizontal();

        // Initializes the Extension
        robotHardware.setInitializeExtension();

        // Lowers the wrist.
        Action action = new SequentialAction(
                new WaitForTime(500),
                new InstantAction(() -> robotHardware.lowerWrist())
        );
        robotHardware.addAction(action);

        // Get current and previous gamepads.
        currentGamepad = new Gamepad();
        previousGamepad = new Gamepad();

        // Reverse the robot.
        robotHardware.setReversed(true);

        robotHardware.initializeLights();

        // While the op mode is active...
        while (opModeIsActive()) {
            Condition sp = new Conditions.SpecimenPark(this);
            // Update the gamepads.
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            // If the right trigger is down...
            /*if (currentGamepad.right_trigger > TRIGGER_THRESHOLD) {

                // Handle debug mode.
                handleDebugMode();

            }

            // Otherwise (if the right trigger is up)...
            else {

                // Handle normal mode.
                handleNormalMode();

            }*/
            handleNormalMode();
            // Update the robot hardware.
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

        /*
        // Handle the submersible controls.
        handleSubmersibleControls();

        // If the user pressed y...
        if (currentGamepad.y && !previousGamepad.y) {


            // Toggle wrist.
            robotHardware.toggleWrist();

        }

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
你好！
         */

//        if (currentGamepad.a && !previousGamepad.a){
//            robotHardware.toggleSmallClaw();
//        }
//        if (currentGamepad.b && !previousGamepad.b){
//            robotHardware.toggleBigClaw();
//        }
//        if (currentGamepad.x && !previousGamepad.x){
//            robotHardware.lowerWrist();
//        }
//        if (currentGamepad.y && !previousGamepad.y){
//            robotHardware.setWristWallPosition();
//        }
//        if (currentGamepad.right_bumper && !previousGamepad.right_bumper){
//            robotHardware.raiseWrist();
//        }

        /*
        if (currentGamepad.dpad_up && !previousGamepad.dpad_up){
            robotHardware.swivelSetVertical();
        }
        if (currentGamepad.dpad_left && !previousGamepad.dpad_left){
            robotHardware.swivelSetHorizontal();
        }
        if (currentGamepad.dpad_right && !previousGamepad.dpad_right){
            robotHardware.swivelSetClip();
        }
        */

//        if (currentGamepad.dpad_down && !previousGamepad.dpad_down){
//            robotHardware.setArmGroundPosition();
//        }
//        if (currentGamepad.dpad_left && !previousGamepad.dpad_left){
//            robotHardware.setArmWallPosition();
//        }
//        if (currentGamepad.dpad_up && !previousGamepad.dpad_up){
//            robotHardware.setArmHighBasketPosition();
//        }
//        if (currentGamepad.dpad_right && !previousGamepad.dpad_right){
//            robotHardware.setArmHighChamberPosition();
//        }
//        if (currentGamepad.back && !previousGamepad.back){
//            robotHardware.setArmSubmersiblePosition();
//        }
//
//        if (currentGamepad.left_bumper && !previousGamepad.left_bumper){
//            robotHardware.setLiftGroundPosition();
//        }
//        if (currentGamepad.left_stick_button && !previousGamepad.left_stick_button){
//           robotHardware.setLiftChamberPosition();
//            robotHardware.minimumExtension();
//        }
//        if (currentGamepad.right_stick_button && !previousGamepad.right_stick_button){
//          robotHardware.setLiftHighBasketPosition();
//            robotHardware.fullExtension();
//        }



    }

    // Handles normal mode.
    private void handleNormalMode() {

        // Set debugging to false
        //////////////////////////////////////////////////////////////////////

        // Set debugging to false
        robotHardware.setDebugging(false);

        // Stop rezeroing.
        //////////////////////////////////////////////////////////////////////

        // Stop rezeroing.
        robotHardware.stopArmRezeroing();
        robotHardware.stopLiftRezeroing();
        robotHardware.stopSlideRezeroing();

        // Enable turtle mode if appropriate.
        //////////////////////////////////////////////////////////////////////

        // Enable turtle mode if appropriate.
        robotHardware.setTurtleMode(currentGamepad.left_trigger > TRIGGER_THRESHOLD);

        // Chamber
        //////////////////////////////////////////////////////////////////////

        // If the user pressed x...
        if (currentGamepad.x && !previousGamepad.x) {

            // Clear any pending actions.
            robotHardware.clearActions();

            // If the arm is in the wall position...
            if (robotHardware.isArmInWallPosition()) {

                // Move the arm to the chamber position.
                Action action = new ParallelAction(
                        new MoveArm(robotHardware, Arm.CHAMBER_POSITION, true),
                        new SequentialAction(
                                new WaitForTime(500),
                                new InstantAction(() -> robotHardware.swivelSetClip()),
                                new InstantAction(() -> robotHardware.setLiftChamberPosition()),
                                new InstantAction(() -> robotHardware.setWristChamberPosition()),
                                new InstantAction(() -> robotHardware.setChamberExtension())
                        )
                );
                robotHardware.addAction(action);

            }

            // Otherwise (if the arm is not in the wall position)...
            else {

                // Notify the user.
                robotHardware.beep();

            }

        }

        // Claw
        //////////////////////////////////////////////////////////////////////

        // If the user pressed a...
        if (currentGamepad.a && !previousGamepad.a) {

            // Clear any pending actions.
            robotHardware.clearActions();

            // If the arm is in the chamber position...
            if (robotHardware.isArmInChamberPosition()) {

                // Release the specimen.
                Action action = new SequentialAction(
                        new InstantAction(() -> robotHardware.openSmallClaw()),
                        new WaitForTime(200),
                        new InstantAction(() -> robotHardware.setWristWallPosition())
                );
                robotHardware.addAction(action);

            }

            // Otherwise, if the arm is in the basket position...
            else if (robotHardware.isArmInBasketPosition()) {

                // Drop the sample and move the arm to the submersible position.
                Action action = new SequentialAction(
                        new InstantAction(() -> robotHardware.openBigClaw()),
                        new WaitForTime(500),
                        new InstantAction(() -> robotHardware.setWristWallPosition()),
                        new WaitForTime(500),
                        new SequentialAction(
                                new InstantAction(() -> robotHardware.setLiftGroundPosition()),
                                new InstantAction(() -> robotHardware.swivelSetHorizontal()),
                                new InstantAction(() -> robotHardware.setInitializeExtension()),
                                new InstantAction(() -> robotHardware.raiseWrist()),
                                new WaitForTime(500),
                                new MoveArm(robotHardware, Arm.SUBMERSIBLE_HOVER_POSITION, true)
                        )
                );
                robotHardware.addAction(action);

            }

            // Otherwise, if the arm is in the wall position...
            else if (robotHardware.isArmInWallPosition()) {

                // If the big claw is open...
                if(robotHardware.isBigClawOpen()) {

                    // Toggle the small claw.
                    robotHardware.toggleSmallClaw();

                }

                // Otherwise (if the big claw is closed)...
                else {

                    // Open the big claw.
                    robotHardware.openBigClaw();

                }

            }

            // Otherwise (if the arm is in another position)...
            else {

                // Toggle the big claw.
                robotHardware.toggleBigClaw();

            }

        }

        // Wall
        //////////////////////////////////////////////////////////////////////

        // If the user pressed b...
        if (currentGamepad.b && !previousGamepad.b) {

            // Clear any pending actions.
            robotHardware.clearActions();

            // If the arm is in the ground or submersible position...
            if(robotHardware.isArmInGroundPosition() || robotHardware.isArmNearSubmersiblePosition()) {

                // Determine whether the arm is in the ground position.
                boolean fromGround = robotHardware.isArmInGroundPosition();

                // Move the arm to the wall position.
                robotHardware.setLiftGroundPosition();
                robotHardware.setArmWallPosition(fromGround);
                robotHardware.setWristWallPosition();
                robotHardware.swivelSetHorizontal();
                robotHardware.setMinimumExtension();

            }

            // Otherwise (if the arm is in another position)...
            else {

                // Notify the user.
                robotHardware.beep();

            }

        }

        // Basket
        //////////////////////////////////////////////////////////////////////

        // If the user pressed y...
        if (currentGamepad.y && !previousGamepad.y) {

            // Clear any pending actions.
            robotHardware.clearActions();

            // If the arm is in the submersible position...
            if (robotHardware.isArmNearSubmersiblePosition()) {

                // Move the arm to the basket position.
                Action action = new SequentialAction(
                        new InstantAction(() -> robotHardware.setWristWallPosition()),
                        new InstantAction(() -> robotHardware.swivelSetVertical()),
                        new InstantAction(() -> robotHardware.setInitializeExtension()),
                        new WaitForSlide(robotHardware, 3000),
                        new InstantAction(() -> robotHardware.setLiftBasketPosition()),
                        new MoveArm(robotHardware, Arm.BASKET_POSITION, true),
                        new InstantAction(() -> robotHardware.setBasketExtension()),
                        new WaitForTime(500),
                        new InstantAction(() -> robotHardware.setWristBasketPosition())
                );
                robotHardware.addAction(action);

            }

            // Otherwise (if the arm is not in the submersible position)...
            else {

                // Notify the user.
                robotHardware.beep();

            }

        }

        // Swivel
        //////////////////////////////////////////////////////////////////////

        // If the user pressed left bumper...
        if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {

            // Clear any pending actions.
            robotHardware.clearActions();

            // Toggle the swivel.
            robotHardware.toggleSwivel();

        }

        // Extend slide
        //////////////////////////////////////////////////////////////////////

        // If the user is holding dpad right and the arm is near the submersible position...
        if(currentGamepad.dpad_right && robotHardware.isArmNearSubmersiblePosition()) {

            // Clear any pending actions.
            robotHardware.clearActions();

            // Extend the slide.
            robotHardware.extendSlide();

        }

        // Retract slide
        //////////////////////////////////////////////////////////////////////

        // If the user is holding dpad left and the arm is near the submersible position...
        if(currentGamepad.dpad_left && robotHardware.isArmNearSubmersiblePosition()) {

            // Clear any pending actions.
            robotHardware.clearActions();

            // Retract the slide.
            robotHardware.retractSlide();

        }

        // Submersible
        //////////////////////////////////////////////////////////////////////

        // If the user pressed right bumper...
        if (currentGamepad.right_bumper && !previousGamepad.right_bumper) {

            // Clear any pending actions.
            robotHardware.clearActions();

            // If the arm is in the basket position...
            if (robotHardware.isArmInBasketPosition()) {

                // Notify the user.
                robotHardware.beep();

            }

            // Otherwise, if the arm is in the submersible position...
            else if (robotHardware.isArmInSubmersibleHoverPosition()) {

                // Grab a sample.
                Action action = new SequentialAction(
                        new InstantAction(() -> robotHardware.openBigClaw()),
                        new MoveArm(robotHardware, Arm.SUBMERSIBLE_GRAB_POSITION, false),
                        new WaitForTime(500),
                        new InstantAction(() -> robotHardware.closeBigClaw()),
                        new WaitForTime(500),
                        new MoveArm(robotHardware, Arm.SUBMERSIBLE_HOVER_POSITION, true)
                );
                robotHardware.addAction(action);

            }

            // Otherwise (if the arm is in another position)...
            else {

                // Move the arm to the submersible position.
                robotHardware.setLiftGroundPosition();
                robotHardware.setArmSubmersibleHoverPosition();
                robotHardware.raiseWrist();
                robotHardware.openBigClaw();

            }

        }
    }
}