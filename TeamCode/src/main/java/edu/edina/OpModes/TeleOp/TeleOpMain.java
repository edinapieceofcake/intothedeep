package edu.edina.OpModes.TeleOp;

import static edu.edina.OpModes.Autonomous.AutoSample.lastPose;
import static edu.edina.OpModes.TeleOp.TeleOpForScrimmage.MAXIMUM_SLIDE_EXTENSION_IN_SUBMERSIBLE;

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

        // Open the claw.
        robotHardware.openClaw();

        // Lowers the wrist.
        robotHardware.lowerWrist();

        // Resets the swivel.
        robotHardware.swivelSetHorizontal();

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
            if (currentGamepad.right_trigger > TRIGGER_THRESHOLD) {

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
        if (currentGamepad.right_bumper && !previousGamepad.right_bumper) {
            robotHardware.setLiftGroundPosition();
            robotHardware.setArmGroundPosition();
            robotHardware.lowerWrist();
            robotHardware.swivelSetHorizontal();
        }
        if (currentGamepad.x && !previousGamepad.x) {
            Action action = new ParallelAction(
                    new MoveArm(robotHardware, Arm.HIGH_CHAMBER_POSITION, false),
                    new SequentialAction(
                            new WaitForTime(500),
                            new InstantAction(() -> robotHardware.swivelSetClip()),
                            new InstantAction(() -> robotHardware.setLiftChamberPosition()),
                            new InstantAction(() -> robotHardware.setWristHighChamberPosition())

                    )


                    );
            robotHardware.addAction(action);
        }
        if (currentGamepad.a && !previousGamepad.a) {
            if (robotHardware.isLiftInChamberPosition()) {
                Action action = new SequentialAction(
                        new InstantAction(() -> robotHardware.toggleSmallClaw()),
                        new WaitForTime(500),
                        new InstantAction(() -> robotHardware.setWristWallPosition())
                );
                robotHardware.addAction(action);
            }
            else {
                robotHardware.toggleSmallClaw();
            }
        }
        if (currentGamepad.b && !previousGamepad.b) {
            robotHardware.setLiftGroundPosition();
            robotHardware.setArmWallPosition();
            robotHardware.setWristWallPosition();
            robotHardware.swivelSetHorizontal();
        }
        if (currentGamepad.left_bumper && !previousGamepad.left_bumper){
           robotHardware.swivelSetClip();
        }
        if (currentGamepad.dpad_left && !previousGamepad.dpad_left){
            robotHardware.swivelSetHorizontal();
        }
        if (currentGamepad.dpad_up && !previousGamepad.dpad_up){
            robotHardware.swivelSetVertical();
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
            // Used for A Test
            // ScoreSpecimen();

            // Clear any pending actions.
            robotHardware.clearActions();
//
//            // Raise the sample.
            robotHardware.raiseSample();

        }

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

        // Ground
        //////////////////////////////////////////////////////////////////////

        // If user pressed right bumper...
        if (currentGamepad.right_bumper && !previousGamepad.right_bumper) {

            // If the robot is in the basket position...
            if (robotHardware.isArmInHighBasketPosition() && robotHardware.isLiftInHighBasketPosition()) {

                // Notify the user.
                robotHardware.beep();

            } else {

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


                robotHardware.toggleSmallClaw();

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
                robotHardware.setWristHighChamberPosition();

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

        // Wall
        //////////////////////////////////////////////////////////////////////

        // If user pressed b...
        if (currentGamepad.b && !previousGamepad.b) {

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

        // Toggle swivel
        //////////////////////////////////////////////////////////////////////

        // If the user tapped left bumper...
        if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {

            // Toggle the swivel.
            robotHardware.toggleSwivel();

        }
    }

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

    private Action goToSub() {
//        Pose2d startAfterAutoPose = new Pose2d(-50, -50, 1.0 / 4 * Math.PI);
//        MecanumDrive drive = new MecanumDrive(hardwareMap, startAfterAutoPose);
        // todo Tune This
        Pose2d subPose = new Pose2d(-30, -30, 2.0 / 4 * Math.PI);
        Action driveFromstartAfterAutoPoseToSub = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(subPose.position, subPose.heading)
                .build();
        Action action = new ParallelAction(
                new KillSwitchAction(robotHardware, () -> !currentGamepad.y),
                driveFromstartAfterAutoPoseToSub,
                new InstantAction(() -> robotHardware.setArmSubmersiblePosition()),
                new InstantAction(() -> robotHardware.setMinimumExtension())
        );
        return action;

    }

    private Action grabSample() {
        Pose2d subPose = new Pose2d(-30, -30, 1.0 / 4 * Math.PI);
//        MecanumDrive drive = new MecanumDrive(hardwareMap, subPose);
//        // todo Tune This
//        Pose2d subGrabPose = new Pose2d(-35, -30, 2.0 / 4 * Math.PI);
        Action backup = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(subPose.position, subPose.heading)
                .build();
        Action action = new SequentialAction(
                new KillSwitchAction(robotHardware, () -> !currentGamepad.y),
                new InstantAction(() -> robotHardware.decrementArmPosition()),
                new WaitForHardware(robotHardware, 3000),
                new InstantAction(() -> robotHardware.closeClaw()),
                new WaitForHardware(robotHardware, 3000),
                new InstantAction(() -> robotHardware.incrementArmPosition()),
                new WaitForHardware(robotHardware, 3000),
                backup,
                new WaitForHardware(robotHardware, 3000)

        );
        return action;

    }

    private Action gotToHighBasket() {
        // TODO Tune This
//        Pose2d subGrabPose = new Pose2d(-35, -30, 2.0 / 4 * Math.PI);
//        MecanumDrive drive = new MecanumDrive(hardwareMap, subGrabPose);
        Pose2d HighBasket = new Pose2d(-55, -50, 4.0 / 4 * Math.PI);
        Action driveFromGrabPoseToHighBasket = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(HighBasket.position, HighBasket.heading)
                .build();
        Action action = new SequentialAction(
                new KillSwitchAction(robotHardware, () -> !currentGamepad.y),
                new InstantAction(() -> robotHardware.setArmSubmersiblePosition()),
                new InstantAction(() -> robotHardware.setMinimumExtension()),
                driveFromGrabPoseToHighBasket,
                new ParallelAction(
                        new MoveArm(robotHardware, Arm.HIGH_BASKET_POSITION, false),
                        new InstantAction(() -> robotHardware.setHighBasketExtension()),
                        new SequentialAction(
                                new WaitForTime(500),
                                new InstantAction(() -> robotHardware.setLiftHighBasketPosition())
                        )
                ),
                new InstantAction(() -> robotHardware.setWristHighBasketPosition()),
                new SequentialAction(
                        new InstantAction(() -> robotHardware.openClaw()),
                        new WaitForTime(500),
                        new ParallelAction(
                                new SequentialAction(
                                        new WaitForTime(200),
                                        new MoveArm(robotHardware, Arm.GROUND_POSITION, false)
                                ),
                                new SequentialAction(
                                        new InstantAction(() -> robotHardware.lowerWrist()),
                                        new WaitForTime(200),
                                        new InstantAction(() -> robotHardware.setWristHighBasketPosition()),
                                        new InstantAction(() -> robotHardware.setMinimumExtension()),
                                        new InstantAction(() -> robotHardware.swivelSetHorizontal()),
                                        new InstantAction(() -> robotHardware.setLiftGroundPosition()),
                                        new WaitForTime(500),
                                        new InstantAction(() -> robotHardware.lowerWrist())
                                )
                        ),
                        new WaitForHardware(robotHardware, 3500)
                )
        );

        // Return the action.
        return action;


    }

    private void killswitch() {
        robotHardware.clearActions();
        robotHardware.stopDrivetrain();
    }

    private void ScoreSpecimen() {
        Action scoreSepcimenAction = new ParallelAction(
                new SpecimenPark(robotHardware, autoCondition),
                new RaiseLift(robotHardware, 12),
                new KillSwitchAction(robotHardware, () -> !currentGamepad.y)
        );
        robotHardware.addAction(scoreSepcimenAction);
    }

}