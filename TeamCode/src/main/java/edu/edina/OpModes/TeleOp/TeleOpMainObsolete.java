package edu.edina.OpModes.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import edu.edina.Libraries.Actions.Ascent;
import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.Robot.Arm;
import edu.edina.Libraries.Robot.Light;
import edu.edina.Libraries.Robot.MoveArm;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.RobotMode;
import edu.edina.Libraries.Robot.WaitForTime;

// Main tele op mode
@Config
@TeleOp
public class TeleOpMainObsolete extends LinearOpMode {

    /*

    Robot Controls

    Gamepad 1
        - left stick = move robot
        - right stick = rotate robot
        - a = score in basket, grab from wall
        - x = chamber
        - left bumper = hold for turtle
        - right trigger = toggle ascend

    Gamepad 2

        Normal Mode
        - left stick = move robot
        - right stick = rotate robot
        - y = basket
        - b = wall
        - x = submersible
        - left bumper = toggle swivel
        - left trigger = clear actions
        - dpad left = retract slide
        - dpad right = extend slide
        - dpad up = raise arm
        - dpad down = lower arm

        Debug Mode (hold right trigger)
        - a = rezero arm
        - x = rezero slide
        - b = rezero lift
        - left bumper = toggle tall walls
        - dpad left = toggle big claw
        - dpad right = toggle small claw

    */

    // Trigger threshold
    public static double TRIGGER_THRESHOLD = 0.5;

    private Light light;

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

    // Robot mode
    private RobotMode robotMode = RobotMode.INITIALIZE;

    // Previous extension target position
    private int previousExtensionTargetPosition;

    // Runs the op mode.
    public void runOpMode() throws InterruptedException {

        // Get hardware.
        robotHardware = new RobotHardware(this);

        robotHardware.initializeLights();

        light = robotHardware.getLight();

        // Prompt the user to press start.
        robotHardware.log("Waiting for start...");

        // Wait for the user to press start.
        waitForStart();

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

        //robotHardware.initializeLights();

        // While the op mode is active...
        while (opModeIsActive()) {

            // Update the gamepads.
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            // If the right trigger is down...
            if (currentGamepad2.right_trigger > TRIGGER_THRESHOLD) {

                // Handle debug mode.
                handleDebugMode();

            }

            // Otherwise (if the right trigger is up)...
            else {

                // Handle normal mode.
                handleNormalMode();

            }

            // Get the tall walls value.
            boolean tallWalls = robotHardware.getTallWalls();

            // Construct a walls string.
            String walls = tallWalls ? "Tall" : "Short";

            // Display main telemetry.
            telemetry.addData("Main", "====================");
            telemetry.addData("- Mode", robotMode);
            telemetry.addData("- Walls", walls);

            // Update the robot hardware.
            robotHardware.update();

            // Update the telemetry.
            telemetry.update();

        }

    }

    // Handles debug mode.
    private void handleDebugMode() {

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

        // Toggle tall walls
        //////////////////////////////////////////////////////////////////////

        // If the user pressed left bumper...
        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {

            // Toggle the tall walls value.
            robotHardware.toggleTallWalls();

        }

        // Toggle big claw
        //////////////////////////////////////////////////////////////////////

        // If the user pressed dpad left...
        if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {

            // Toggle the big claw.
            robotHardware.toggleBigClaw();

        }

        // Toggle small claw
        //////////////////////////////////////////////////////////////////////

        // If the user pressed dpad right...
        if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {

            // Toggle the small claw.
            robotHardware.toggleSmallClaw();

        }

    }

    // Handles normal mode.
    private void handleNormalMode() throws InterruptedException {

        // Stop rezeroing.
        //////////////////////////////////////////////////////////////////////

        // Stop rezeroing.
        robotHardware.stopArmRezeroing();
        robotHardware.stopLiftRezeroing();
        robotHardware.stopSlideRezeroing();

        // Enable turtle mode if appropriate.
        //////////////////////////////////////////////////////////////////////

        // Enable turtle mode if appropriate.
        robotHardware.setTurtleMode(currentGamepad1.left_bumper);

        // Chamber
        //////////////////////////////////////////////////////////////////////

        // If the user pressed x...
        if (currentGamepad1.x && !previousGamepad1.x) {

            // If the robot is in wall mode...
            if (robotMode == RobotMode.WALL) {

                // Move the arm to the chamber position.
                Action action = robotHardware.raiseToChamber();
                robotHardware.addAction(action);

                // Set the robot mode to chamber.
                robotMode = RobotMode.CHAMBER;

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
        if (currentGamepad1.a && !previousGamepad1.a) {

            // If the robot is in basket mode...
            if (robotMode == RobotMode.BASKET) {

                // Score the sample and lower the arm.
                robotHardware.addAction(robotHardware.scoreSampleAndLower(true));

                // Set the robot mode to submersible.
                robotMode = RobotMode.SUBMERSIBLE;

            }

            // Otherwise, if the robot is in wall mode...
            else if (robotMode == RobotMode.WALL) {
                // Toggle the big claw.
                robotHardware.toggleSmallClaw();

            }

            // Otherwise (if the robot is in another mode)...
            else {

                // Notify the user.
                robotHardware.beep();

            }

        }

        // Wall
        //////////////////////////////////////////////////////////////////////

        // If the user pressed b...
        if (currentGamepad2.b && !previousGamepad2.b) {

            // Get the tall walls value.
            boolean tallWalls = robotHardware.getTallWalls();

            // If the robot is in chamber mode...
            if (robotMode == RobotMode.CHAMBER) {

                // Release the specimen.
                Action action = new SequentialAction(

                        // Open the small claw to release the specimen.
                        new InstantAction(() -> robotHardware.openSmallClaw()),

                        // Wait a bit.
                        new WaitForTime(200),

                        // Lower the wrist.
                        new InstantAction(() -> robotHardware.setWristSubmersiblePosition()),

                        // Lower the lift.
                        new InstantAction(() -> robotHardware.setLiftGroundPosition()),

                        // Lower the arm.
                        new MoveArm(robotHardware, Arm.WALL_POSITION, MoveArm.MEDIUM_INCREMENT),

                        // Set the wrist to the wall position.
                        new InstantAction(() -> robotHardware.setWristWallPosition(tallWalls)),

                        // Set the swivel to horizontal.
                        new InstantAction(() -> robotHardware.swivelSetHorizontal()),

                        // Set the robot mode to wall.
                        new InstantAction(() -> {
                            robotMode = RobotMode.WALL;
                        })

                );
                robotHardware.addAction(action);

            }

            // Otherwise, if the robot mode is submersible...
            else if (robotMode == RobotMode.INITIALIZE || robotMode == RobotMode.SUBMERSIBLE) {

                // Move the arm to the wall position.
                Action action = new ParallelAction(
                        new InstantAction(() -> robotHardware.setLiftGroundPosition()),
                        new MoveArm(robotHardware, Arm.WALL_POSITION, MoveArm.FAST_INCREMENT),
                        new InstantAction(() -> robotHardware.setWristWallPosition(tallWalls)),
                        new InstantAction(() -> robotHardware.swivelSetHorizontal()),
                        new InstantAction(() -> robotHardware.setMinimumExtension())
                );
                robotHardware.addAction(action);

                // Set the robot mode to wall.
                robotMode = RobotMode.WALL;

            }

            // Otherwise (if the robot is in a different mode)...
            else {

                // Notify the user.
                robotHardware.beep();

            }

        }

        // Basket
        //////////////////////////////////////////////////////////////////////

        // If the user pressed y...
        if (currentGamepad2.y && !previousGamepad2.y) {

            // If the robot is in submersible mode and the wrist is up...
            if (robotMode == RobotMode.SUBMERSIBLE && !robotHardware.isWristInSubmersiblePosition()) {

                // Raise the sample to the basket.
                robotHardware.addAction(robotHardware.raiseSampleToBasket());

                // Set the robot mode to basket.
                robotMode = RobotMode.BASKET;

            }

            // Otherwise (if the robot is not in submersible mode)...
            else {

                // Notify the user.
                robotHardware.beep();

            }

        }

        // Swivel
        //////////////////////////////////////////////////////////////////////

        // If the user pressed left bumper...
        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {

            // Toggle the swivel.
            robotHardware.toggleSwivel();

        }

        // Extend slide
        //////////////////////////////////////////////////////////////////////

        // If the user is holding dpad right...
        if (currentGamepad2.dpad_right) {

            // If the robot is in submersible mode...
            if (robotMode == RobotMode.SUBMERSIBLE) {

                // Extend the slide.
                robotHardware.extendSlide();

            }

            // Otherwise (if the robot is not in submersible mode)...
            else {

                // Notify the user.
                robotHardware.beep();

            }

        }

        // Retract slide
        //////////////////////////////////////////////////////////////////////

        // If the user is holding dpad left...
        if (currentGamepad2.dpad_left) {

            // If the robot is in submersible mode...
            if (robotMode == RobotMode.SUBMERSIBLE) {

                // Retract the slide.
                robotHardware.retractSlide();

            }

            // Otherwise (if the robot is not in submersible mode)...
            else {

                // Notify the user.
                robotHardware.beep();

            }

        }

        // Submersible
        //////////////////////////////////////////////////////////////////////

        // If the user pressed x...
        if (currentGamepad2.x && !previousGamepad2.x) {

            // If the robot is in submersible mode...
            if (robotMode == RobotMode.SUBMERSIBLE) {

                // If the wrist is down...
                if (robotHardware.isWristInSubmersiblePosition()) {

                    // Remember the extension's target position.
                    previousExtensionTargetPosition = robotHardware.getExtensionTargetPosition();

                    // Grab a sample and retract.
                    Action action = new SequentialAction(
                            new InstantAction(() -> robotHardware.openSmallClaw()),
                            new MoveArm(robotHardware, Arm.SUBMERSIBLE_GRAB_POSITION, MoveArm.SLOW_INCREMENT),
                            new WaitForTime(100),
                            new InstantAction(() -> robotHardware.closeSmallClaw()),
                            new WaitForTime(50),
                            new InstantAction(() -> robotHardware.setWristWallPosition(true)),
                            new InstantAction(() -> robotHardware.setMinimumExtension()),
                            new MoveArm(robotHardware, Arm.SUBMERSIBLE_HOVER_POSITION, MoveArm.FAST_INCREMENT)
                    );
                    robotHardware.addAction(action);

                }

                // Otherwise (if the wrist is up)...
                else {

                    // Reach back into the submersible.
                    Action action = new SequentialAction(
                            new InstantAction(() -> robotHardware.openSmallClaw()),
                            new InstantAction(() -> robotHardware.setExtensionTargetPosition(previousExtensionTargetPosition)),
                            new InstantAction(() -> robotHardware.setWristSubmersiblePosition())
                    );
                    robotHardware.addAction(action);

                }

            }

            // Otherwise, if the robot is in chamber mode...
            else if (robotMode == RobotMode.CHAMBER) {

                Pose2d startPose = new Pose2d(0, 0, Math.toRadians(270));
                Pose2d endPose = new Pose2d(0, -7.5, Math.toRadians(270));

                MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

                Action backUp = drive.actionBuilder(startPose)
                        .strafeTo(endPose.position)
                        .build();

                Action moveForward = drive.actionBuilder(endPose)
                        .strafeTo(startPose.position)
                        .build();

                // Go to submersible from chamber.
                Action action = new SequentialAction(
                        new InstantAction(() -> robotHardware.disableManualDriving()),
                        new InstantAction(() -> robotHardware.openSmallClaw()),
                        new WaitForTime(200),
                        new InstantAction(() -> robotHardware.setWristSubmersiblePosition()),
                        new ParallelAction(
                                backUp,
                                new SequentialAction(
                                        new WaitForTime(800),
                                        new InstantAction(() -> robotHardware.setLiftGroundPosition())
                                ),
                                new SequentialAction(
                                        new WaitForTime(1200),
                                        new InstantAction(() -> robotHardware.setWristWallPosition(true))
                                ),
                                new SequentialAction(
                                        new WaitForTime(1600),
                                        new InstantAction(() -> robotHardware.setSubmersibleExtension())
                                ),
                                new InstantAction(() -> robotHardware.openBigClaw()),
                                new MoveArm(robotHardware, Arm.SUBMERSIBLE_ENTER_POSITION, MoveArm.FAST_INCREMENT)
                        ),
                        new ParallelAction(
                                moveForward,
                                new SequentialAction(
                                        new WaitForTime(600),
                                        new InstantAction(() -> robotHardware.setWristSubmersiblePosition())
                                )
                        ),
                        new InstantAction(() -> robotHardware.enableManualDriving())
                );
                robotHardware.addAction(action);

                // Set the robot mode to submersible.
                robotMode = RobotMode.SUBMERSIBLE;

            }

            // Otherwise, if the robot is in initialize mode...
            else if (robotMode == RobotMode.INITIALIZE || robotMode == RobotMode.WALL) {

                // Move the arm to the submersible position.
                Action action = new SequentialAction(
                        new InstantAction(() -> robotHardware.setLiftGroundPosition()),
                        new InstantAction(() -> robotHardware.setWristWallPosition(true)),
                        new InstantAction(() -> robotHardware.openSmallClaw()),
                        new MoveArm(robotHardware, Arm.SUBMERSIBLE_ENTER_POSITION, MoveArm.FAST_INCREMENT),
                        new InstantAction(() -> robotHardware.setSubmersibleExtension()),
                        new InstantAction(() -> robotHardware.setWristSubmersiblePosition())
                );
                robotHardware.addAction(action);

                // Set the robot mode to submersible.
                robotMode = RobotMode.SUBMERSIBLE;

            }

            // Otherwise (if the robot is in another mode)...
            else {

                // Notify the user.
                robotHardware.beep();

            }

        }

        // Decrement arm
        //////////////////////////////////////////////////////////////////////

        // If the user is holding dpad up...
        if (currentGamepad2.dpad_up) {

            // If the robot is in initialize or wall mode...
            if (robotMode == RobotMode.INITIALIZE || robotMode == RobotMode.WALL) {

                // Increment the arm position.
                robotHardware.incrementArmPosition();

            }
            else if (robotMode == RobotMode.BASKET) {
                if (robotHardware.getCorrectedArmPosition() > 1600) {
                    robotHardware.decrementArmPosition();
                }else {
                    robotHardware.beep();
                }
            }

            // Otherwise (if the robot is in another mode)...
            else {

                // Decrement the arm position.
                robotHardware.decrementArmPosition();

            }

        }

        // Increment arm
        //////////////////////////////////////////////////////////////////////

        // If the user is holding dpad down...
        if (currentGamepad2.dpad_down) {

            // If the robot is in initialize or wall mode...
            if (robotMode == RobotMode.INITIALIZE || robotMode == RobotMode.WALL) {

                // Derement the arm position.
                robotHardware.decrementArmPosition();

            }

            // Otherwise (if the robot is in another mode)...
            else {

                // Increment the arm position.
                robotHardware.incrementArmPosition();

            }

        }

        // Clear all actions
        //////////////////////////////////////////////////////////////////////

        // If the user pressed left trigger...
        if (currentGamepad2.left_trigger > TRIGGER_THRESHOLD && previousGamepad2.left_trigger <= TRIGGER_THRESHOLD) {

            // Clear actions.
            robotHardware.clearActions();

        }

        // Ascend
        //////////////////////////////////////////////////////////////////////

        // If the user pressed right trigger...
        if (currentGamepad1.right_trigger > TRIGGER_THRESHOLD && previousGamepad1.right_trigger <= TRIGGER_THRESHOLD) {

            // If the robot is in submersible mode...
            if(robotMode == RobotMode.SUBMERSIBLE) {

                // Toggle ascent.
                robotHardware.turnOnWave();
                robotHardware.setMinimumExtension();
                robotHardware.setArmSubmersibleEnterPosition();
                robotHardware.incrementArmPosition();
                ascent();

            }

            // Otherwise (if the robot is not in submersible mode)...
            else {

                // Notify the user.
                robotHardware.beep();

            }

        }

    }

    // It is important to not call robotHardware.update() because we turn off the hardware,
    // and it will cause several exceptions.
    //////////////////////////////////////////////////////////////////////////
    public void ascent() throws InterruptedException {
        Ascent a = new Ascent(robotHardware);
        robotHardware.addAction(new SequentialAction(
                new InstantAction(a::turnOffServos),
                new ParallelAction(
                        a.raiseLift(),
                        new SequentialAction(
                                new WaitForTime(200),
                                a.moveArmToAllowLiftsToRaise()
                        ),
                        new SequentialAction(
                                a.checkLiftHeight(),
                                a.lowerHook(),
                                new WaitForTime(200), // 200 normal, testing without servo
                                a.turnOffHook()
                        )
                ),
                new ParallelAction(
                        a.lowerLift(),
                        a.moveArmToAllowLiftsToLower()
                ),
                a.waitForHook(),
                new ParallelAction(
                        a.hang(),
                        a.engageHook()
                )
        ));

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            light.update(true, false);

            if (a.getAbort() || (currentGamepad1.x && !previousGamepad1.x)) {
                robotHardware.turnOffWave();
                a.turnOnServos();
                return;
            } else {
                robotHardware.runActions();
                robotHardware.getArm().update();
                robotHardware.getSlide().update();
                robotHardware.drivetrain.update2();
            }
        }
    }
}