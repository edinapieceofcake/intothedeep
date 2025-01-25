package edu.edina.OpModes.TeleOp;

import static edu.edina.OpModes.Autonomous.AutoSample.BASKET_HEADING;
import static edu.edina.OpModes.Autonomous.AutoSample.BASKET_TANGENT;
import static edu.edina.OpModes.Autonomous.AutoSample.BASKET_X;
import static edu.edina.OpModes.Autonomous.AutoSample.BASKET_Y;
import static edu.edina.OpModes.Autonomous.AutoSample.lastPose;
import static edu.edina.OpModes.TeleOp.TeleOpForScrimmage.TRIGGER_THRESHOLD;

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
import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.Robot.Arm;
import edu.edina.Libraries.Robot.MoveArm;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.WaitForTime;
import edu.edina.OpModes.Autonomous.AutoSample;

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
    - dpad left = retract slide
    - dpad right = extend slide
    - back = toggle ascend (unimplemented)

    Debug Mode (hold right trigger)

    - a = rezero arm 
    - x = rezero slide
    - b = rezero lift
    - y = score sample

    */

    // Submersible pose
    public static double SUBMERSIBLE_X = -24;
    public static double SUBMERSIBLE_Y = -6;
    public static double SUBMERSIBLE_HEADING = Math.toRadians(180);
    public static double SUBMERSIBLE_TANGENT = Math.toRadians(0);

    // Ascending value
    private boolean ascending;

    // Current gamepad
    private Gamepad currentGamepad1 = new Gamepad();

    // Previous gamepad
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad currentGamepad2 = new Gamepad();

    // Previous gamepad
    private Gamepad previousGamepad2 = new Gamepad();

    // Robot hardware
    private RobotHardware robotHardware;
    private MecanumDrive drive;
    private Condition autoCondition;


    // Runs the op mode.
    public void runOpMode() throws InterruptedException {

        // Get hardware.
        robotHardware = new RobotHardware(this);

        // DEBUG
        lastPose = new Pose2d(AutoSample.START_X, AutoSample.START_Y, AutoSample.START_HEADING);

        // Construct a drive interface.
        drive = new MecanumDrive(hardwareMap, lastPose);

        // Prompt the user to press start.
        robotHardware.log("Waiting for start...");

        // Wait for the user to press start.
        waitForStart();
        autoCondition = null;

        // Open the claws.
        robotHardware.openClaws();

        // Resets the swivel.
        robotHardware.swivelSetHorizontal();

        // Set the wrist to the submersible position.
        robotHardware.setWristSubmersiblePosition();

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

            Condition sp = new Conditions.SpecimenPark(this);

            // Update the gamepads.
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            // If the right trigger is down...
            if (currentGamepad1.right_trigger > TRIGGER_THRESHOLD) {

                // Handle debug mode.
                handleDebugMode();

            }

            // Otherwise (if the right trigger is up)...
            else {

                // Handle normal mode.
                handleNormalMode();

            }

            // Update the pose estimate.
            drive.updatePoseEstimate();

            // Display the pose.
            telemetry.addData("Pose", "====================");
            telemetry.addData("- X", drive.pose.position.x);
            telemetry.addData("- Y", drive.pose.position.y);
            telemetry.addData("- Heading", Math.toDegrees(drive.pose.heading.toDouble()));

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

        // Arm rezeroing
        //////////////////////////////////////////////////////////////////////

        // If the user pressed a...
        if (currentGamepad2.a && !previousGamepad2.a) {

            // Start arm rezeroing.
            robotHardware.startArmRezeroing();

        }

        // If the user released a...
        if (!currentGamepad2.a && previousGamepad2.a) {

            // Stop arm rezeroing.
            robotHardware.stopArmRezeroing();

        }

        // Slide rezeroing
        //////////////////////////////////////////////////////////////////////

        // If the user pressed x...
        if (currentGamepad2.x && !previousGamepad2.x) {

            // Start slide rezeroing.
            robotHardware.startSlideRezeroing();

        }

        // If the user released x...
        if (!currentGamepad2.x && previousGamepad2.x) {

            // Start slide rezeroing.
            robotHardware.stopSlideRezeroing();

        }

        // Lift rezeroing
        //////////////////////////////////////////////////////////////////////

        // If the user pressed b...
        if (currentGamepad2.b && !previousGamepad2.b) {

            // Start lift rezeroing.
            robotHardware.startLiftRezeroing();

        }

        // If the user released b...
        if (!currentGamepad2.b && previousGamepad2.b) {

            // Start lift rezeroing.
            robotHardware.stopLiftRezeroing();

        }

        // Automatic sample scoring
        //////////////////////////////////////////////////////////////////////

        // If the user pressed a...
        if (currentGamepad2.y && !previousGamepad2.y) {

            // If there are running actions...
            if(robotHardware.hasRunningActions()) {

                // Clear the running actions.
                robotHardware.clearActions();

            }

            // Otherwise (if there are no running actions)...
            else {

                // Get the robot's current pose.
                Pose2d currentPose = drive.pose;

                // Construct a basket pose.
                Pose2d basketPose = new Pose2d(BASKET_X, BASKET_Y, BASKET_HEADING);

                // Construct a submersible pose.
                Pose2d submersiblePose = new Pose2d(SUBMERSIBLE_X, SUBMERSIBLE_Y, SUBMERSIBLE_HEADING);

                // Construct an action for driving from the current position to the basket.
                Action driveFromCurrentToBasket = drive.actionBuilder(currentPose)
                        .setReversed(false)
                        .splineTo(basketPose.position, BASKET_TANGENT)
                        .build();

                // Construct an action for driving from the basket to the submersible.
                Action driveFromBasketToSubmersible = drive.actionBuilder(basketPose)
                        .setReversed(true)
                        .splineTo(submersiblePose.position, SUBMERSIBLE_TANGENT)
                        .build();

                // Construct a score action.
                Action scoreAction = new SequentialAction(

                        // Disable manual driving.
                        new InstantAction(() -> robotHardware.disableManualDriving()),

                        // Raise the arm while driving to the basket
                        new ParallelAction(

                                // Drive from the current position to the basket.
                                driveFromCurrentToBasket,

                                new SequentialAction(
                                        // Wait until it has pulled out of the submersible
                                        new WaitForTime(1000),

                                        // Raise the sample to the basket.
                                        robotHardware.raiseSampleToBasket()
                                )

                        ),

                        // Score the sample.
                        robotHardware.scoreSample(),

                        // Lower the arm while driving to the submersible.
                        new ParallelAction(

                                // Lower the arm.
                                robotHardware.lowerArmFromBasket(true),

                                // Drive to the submersible.
                                driveFromBasketToSubmersible

                        ),

                        // Enable manual driving.
                        new InstantAction(() -> robotHardware.enableManualDriving())

                );

                // Score the sample.
                robotHardware.addAction(scoreAction);

            }

        }

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
        robotHardware.setTurtleMode(currentGamepad1.left_trigger > TRIGGER_THRESHOLD);

        // Chamber
        //////////////////////////////////////////////////////////////////////

        // If the user pressed x...
        if (currentGamepad2.x && !previousGamepad2.x) {

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
        if (currentGamepad2.a && !previousGamepad2.a) {

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

                // Score the sample and lower the arm.
                robotHardware.addAction(robotHardware.scoreSampleAndLower(true));

            }

            // Otherwise, if the arm is in the wall position...
            else if (robotHardware.isArmInWallPosition()) {

                // If the big claw is open...
                if (robotHardware.isBigClawOpen()) {

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
        if (currentGamepad2.b && !previousGamepad2.b) {

            // Clear any pending actions.
            robotHardware.clearActions();

            // If the arm is in the ground or submersible position...
            if (robotHardware.isArmInGroundPosition() || robotHardware.isArmNearSubmersiblePosition()) {

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
        if (currentGamepad2.y && !previousGamepad2.y) {

            // Clear any pending actions.
            robotHardware.clearActions();

            // If the arm is in the submersible position...
            if (robotHardware.isArmNearSubmersiblePosition()) {

                robotHardware.addAction(robotHardware.raiseSampleToBasket());

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
        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {

            // Clear any pending actions.
            robotHardware.clearActions();

            // Toggle the swivel.
            robotHardware.toggleSwivel();

        }

        // Extend slide
        //////////////////////////////////////////////////////////////////////

        // If the user is holding dpad right and the arm is near the submersible position...
        if (currentGamepad2.dpad_right && robotHardware.isArmNearSubmersiblePosition()) {

            // Clear any pending actions.
            robotHardware.clearActions();

            // Extend the slide.
            robotHardware.extendSlide();

        }

        // Retract slide
        //////////////////////////////////////////////////////////////////////

        // If the user is holding dpad left and the arm is near the submersible position...
        if (currentGamepad2.dpad_left && robotHardware.isArmNearSubmersiblePosition()) {

            // Clear any pending actions.
            robotHardware.clearActions();

            // Retract the slide.
            robotHardware.retractSlide();

        }

        // Submersible
        //////////////////////////////////////////////////////////////////////

        // If the user pressed right bumper...
        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {

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
                robotHardware.setWristSubmersiblePosition();
                robotHardware.openBigClaw();

            }

        }
        if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up && robotHardware.isArmNearSubmersiblePosition()) {

            // Decrement the arm position.
            robotHardware.decrementArmPosition();

        }

        // Increment arm
        //////////////////////////////////////////////////////////////////////

        // If the user tapped dpad down...
        if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down && robotHardware.isArmNearSubmersiblePosition()) {

            // Increment the arm position.
            robotHardware.incrementArmPosition();

        }
        /*
        // Submersible
        //////////////////////////////////////////////////////////////////////

        if (robotHardware.isArmNearSubmersiblePosition()) {

            handleSubmersibleControls();

        }

        // If the user pressed dpad up...
        else if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {

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
        */
    }
    /*
    private void handleSubmersibleControls() {

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

    }
    */
}