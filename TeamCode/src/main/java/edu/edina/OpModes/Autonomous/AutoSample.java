package edu.edina.OpModes.Autonomous;

import static edu.edina.Libraries.Robot.RobotHardware.YELLOW_SQUARE;
import static edu.edina.Libraries.Robot.RobotHardware.getBanner;
import static edu.edina.Libraries.Robot.RobotHardware.prompt;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.Robot.Arm;
import edu.edina.Libraries.Robot.GrabHumanSample;
import edu.edina.Libraries.Robot.MoveArm;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.SampleType;
import edu.edina.Libraries.Robot.WaitForHardware;
import edu.edina.Libraries.Robot.WaitForTime;

@Config
@Autonomous(preselectTeleOp = "TeleOpMain")
public class AutoSample extends LinearOpMode {

    // Start pose
    public static double START_X = -35;
    public static double START_Y = -61;
    public static double START_HEADING = Math.toRadians(180);

    // Basket pose
    public static double BASKET_X = -58;
    public static double BASKET_Y = -58;
    public static double BASKET_HEADING = Math.toRadians(225);

    // First spike mark pose
    public static double FIRST_SPIKE_MARK_X_END = -48;
    public static double FIRST_SPIKE_MARK_Y_END = -37.5;
    public static double FIRST_SPIKE_MARK_X_BEGINNING = -48;
    public static double FIRST_SPIKE_MARK_Y_BEGINNING = -34;
    public static double FIRST_SPIKE_MARK_HEADING = Math.toRadians(270);

    // Second spike mark pose
    public static double SECOND_SPIKE_MARK_X_END = -58;
    public static double SECOND_SPIKE_MARK_Y_END = -39.5;
    public static double SECOND_SPIKE_MARK_X_BEGINNING = -58;
    public static double SECOND_SPIKE_MARK_Y_BEGINNING = -36;
    public static double SECOND_SPIKE_MARK_HEADING = FIRST_SPIKE_MARK_HEADING;

    // Third spike mark pose
    public static double THIRD_SPIKE_MARK_X_END = -54;
    public static double THIRD_SPIKE_MARK_Y_END = -25.5;
    public static double THIRD_SPIKE_MARK_X_BEGINNING = -54;
    public static double THIRD_SPIKE_MARK_Y_BEGINNING = -21;
    public static double THIRD_SPIKE_MARK_HEADING = Math.toRadians(0);

    // Human pose
    public static double HUMAN_X = 36.5;
    public static double HUMAN_Y = -53.5;
    public static double FIRST_HUMAN_HEADING = Math.toRadians(0);
    public static double SECOND_HUMAN_HEADING = Math.toRadians(180);

    // Rungs pose
    public static double RUNGS_X = -35;
    public static double RUNGS_Y = -9;
    public static double RUNGS_HEADING = Math.toRadians(0);

    // Chamber pose
    public static double CHAMBER_X = -15;
    public static double CHAMBER_Y = -40;
    public static double CHAMBER_HEADING = Math.toRadians(0);

    // Fast velocity
    public static double FAST_VELOCITY = 40;

    // Medium velocity
    public static double MEDIUM_VELOCITY = 30;

    // Slow velocity
    public static double SLOW_VELOCITY = 15;

    // Robot hardware
    private RobotHardware robotHardware;

    // Grab human sample value
    private GrabHumanSample grabHumanSample;
    private Boolean rungsPark;

    // Runs the op mode.
    @Override
    public void runOpMode() throws InterruptedException {

        // Display launch menu.
        //////////////////////////////////////////////////////////////////////

        // Initialize an input tall walls value.
        Boolean inputTallWalls = null;

        // Initialize gamepads.
        Gamepad currentGamepad = new Gamepad();
        Gamepad previousGamepad = new Gamepad();

        // While we are waiting for a response...
        while (!isStopRequested()) {

            // Update gamepads.
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            // If the grab human sample value is missing...
            if(grabHumanSample == null) {

                // Prompt the user for a grab human sample value.
                prompt(telemetry, "Grab Human Sample", "X = Beginning, A = End, B = Never");

                // If the user pressed x...
                if (currentGamepad.x && !previousGamepad.x) {

                    // Grab the human sample at the beginning.
                    grabHumanSample = GrabHumanSample.BEGINNING;

                }

                // Otherwise, if the user pressed a...
                else if (currentGamepad.a && !previousGamepad.a) {

                    // Grab the human sample at the end.
                    grabHumanSample = GrabHumanSample.END;

                }

                // Otherwise, if the user pressed b...
                else if (currentGamepad.b && !previousGamepad.b) {

                    // Do not gab the human sample.
                    grabHumanSample = GrabHumanSample.NEVER;

                }

            }

            // Otherwise, if the rungs park value is missing...
            else if(rungsPark == null) {

                // Prompt the user for a park value.
                prompt(telemetry, "Park", "X = Chamber, B = Rungs");

                // If the user pressed x...
                if (currentGamepad.x && !previousGamepad.x) {

                    // Park at the chamber.
                    rungsPark = false;

                }

                // Otherwise, if the user pressed b...
                else if (currentGamepad.b && !previousGamepad.b) {

                    // Park at the rungs.
                    rungsPark = true;

                }

            }

            // Otherwise, if the tall walls value is missing...
            else if(inputTallWalls == null) {

                // Prompt the user for a walls value.
                prompt(telemetry, "Walls", "X = Tall, B = Short");

                // If the user pressed x...
                if (currentGamepad.x && !previousGamepad.x) {

                    // Use tall walls.
                    inputTallWalls = true;

                }

                // Otherwise, if the user pressed b...
                else if (currentGamepad.b && !previousGamepad.b) {

                    // Use short walls.
                    inputTallWalls = false;

                }

            }

            // Otherwise (if we are done)...
            else {

                // Hide the launch menu.
                break;

            }

        }

        // If stop is requested...
        if (isStopRequested()) {

            // Exit the method.
            return;

        }

        // Initialize the robot.
        //////////////////////////////////////////////////////////////////////

        // Construct a start pose.
        Pose2d startPose = new Pose2d(START_X, START_Y, START_HEADING);

        // Get hardware.
        robotHardware = new RobotHardware(this, startPose);

        // Indicate that the robot is initializing.
        robotHardware.log("Initializing...");

        // Set the tall walls value.
        robotHardware.setTallWalls(inputTallWalls);

        // Wait for the user to lower the lift.
        robotHardware.waitForLiftDown();

        // Wait for the user to lower the arm.
        robotHardware.waitForArmDown();

        // Close the big claw.
        robotHardware.closeBigClaw();

        // Open the small claw.
        robotHardware.openSmallClaw();

        // Set the wrist to the submersible position.
        robotHardware.setWristSubmersiblePosition();

        // Set the swivel to the horizontal position.
        robotHardware.swivelSetHorizontal();

        // If stop is requested...
        if (isStopRequested()) {

            // Exit the method.
            return;

        }

        // Wait for start.
        //////////////////////////////////////////////////////////////////////

        // Add telemetry.
        addTelemetry();

        // Prompt the user to press start.
        robotHardware.log("Waiting for start...");

        // Wait for the user to press start.
        waitForStart();

        // Run the op mode.
        //////////////////////////////////////////////////////////////////////

        // Indicate that this is running.
        robotHardware.log("Running...");

        // Disable manual driving.
        robotHardware.disableManualDriving();

        // Get a drive interface.
        MecanumDrive drive = robotHardware.getDrive();

        // Set the main action.
        Action mainAction = getMainAction(drive, startPose);

        // Add the action to the robot hardware.
        robotHardware.addAction(mainAction);

        // While the op mode is active...
        while (opModeIsActive()) {

            // Add telemetry.
            addTelemetry();

            // Update the robot hardware.
            robotHardware.update();

            // Update the telemetry.
            telemetry.update();

        }

    }

    // Scores the current sample and then gets the next sample.
    private static Action scoreAndGrab(RobotHardware robotHardware, Action driveFromBasketToSample, Action driveFromSampleToBasket, SampleType sampleType) {

        // Determine whether the sample type is human.
        boolean isHumanSample = sampleType == SampleType.HUMAN;

        // Determine whether the sample type is wall.
        boolean isWallSample = sampleType == SampleType.WALL;

        // Construct an action.
        Action action = new SequentialAction(

                // Score the current sample.
                robotHardware.scoreSample(),

                // Lower the arm while driving to the sample.
                new ParallelAction(

                        // Lower the arm.
                        new SequentialAction(

                                // Lower the arm from the basket.
                                robotHardware.lowerArmFromBasket(!isWallSample, true, true, !isWallSample),

                                // If this the wall sample, finish extending the slide.
                                new InstantAction(() -> robotHardware.setAutoExtension()),

                                // Wait for the arm to settle.
                                new WaitForTime(250),

                                // If this is not the human sample, lower the arm to grab.
                                isHumanSample ?
                                        new SequentialAction() :
                                        new MoveArm(robotHardware, Arm.SUBMERSIBLE_GRAB_POSITION, false),

                                // Wait for the hardware.
                                new WaitForHardware(robotHardware, 1000)

                        ),

                        // Drive to the sample and then lower the arm to grab if appropriate.
                        new SequentialAction(

                                // Drive to the sample.
                                driveFromBasketToSample,

                                // If this is the human sample, lower the arm to grab.
                                isHumanSample ?
                                        new MoveArm(robotHardware, Arm.SUBMERSIBLE_GRAB_POSITION, false) :
                                        new SequentialAction()

                        )

                ),

                // Wait for the arm to settle.
                new WaitForTime(250),

                // Grab the spike mark sample.
                new InstantAction(() -> robotHardware.closeBigClaw()),
                new WaitForTime(250),

                // Drive to the basket and raise the sample.
                new ParallelAction(

                        // Drive to the basket.
                        driveFromSampleToBasket,

                        new SequentialAction(

                                // If this is the wall sample, wait for a bit to clear the wall.
                                new WaitForTime(isWallSample ? 500 : 0),

                                // Raise the sample to the basket.
                                robotHardware.raiseSampleToBasket()

                        )

                )

        );

        // Return the action.
        return action;

    }

    // Gets a main action.
    private Action getMainAction(MecanumDrive drive, Pose2d startPose) {

        // Construct velocity constraints.
        //////////////////////////////////////////////////////////////////////

        // Construct a spike mark velocity constraint.
        VelConstraint spikeMarkVelocityConstraint = (robotPose, _path, _disp) ->
                robotPose.position.y.value() > -40 ? SLOW_VELOCITY : MEDIUM_VELOCITY;

        // Construct a human sample velocity constraint.
        VelConstraint humanSampleVelocityConstraint = (robotPose, _path, _disp) ->
                robotPose.position.x.value() > 20 ? SLOW_VELOCITY : FAST_VELOCITY;

        // Construct a human velocity constraint.
		TranslationalVelConstraint humanVelocityConstraint =
                new TranslationalVelConstraint(FAST_VELOCITY);

        // Construct trajectories.
        //////////////////////////////////////////////////////////////////////

        // Construct a basket pose.
        Pose2d basketPose = new Pose2d(BASKET_X, BASKET_Y, BASKET_HEADING);

        // Get spike mark locations.
        double firstSpikeMarkX = grabHumanSample == GrabHumanSample.BEGINNING ? FIRST_SPIKE_MARK_X_BEGINNING : FIRST_SPIKE_MARK_X_END;
        double firstSpikeMarkY = grabHumanSample == GrabHumanSample.BEGINNING ? FIRST_SPIKE_MARK_Y_BEGINNING : FIRST_SPIKE_MARK_Y_END;
        double secondSpikeMarkX = grabHumanSample == GrabHumanSample.BEGINNING ? SECOND_SPIKE_MARK_X_BEGINNING : SECOND_SPIKE_MARK_X_END;
        double secondSpikeMarkY = grabHumanSample == GrabHumanSample.BEGINNING ? SECOND_SPIKE_MARK_Y_BEGINNING : SECOND_SPIKE_MARK_Y_END;
        double thirdSpikeMarkX = grabHumanSample == GrabHumanSample.BEGINNING ? THIRD_SPIKE_MARK_X_BEGINNING : THIRD_SPIKE_MARK_X_END;
        double thirdSpikeMarkY = grabHumanSample == GrabHumanSample.BEGINNING ? THIRD_SPIKE_MARK_Y_BEGINNING : THIRD_SPIKE_MARK_Y_END;

        // Construct a first spike mark pose.
        Pose2d firstSpikeMarkPose = new Pose2d(firstSpikeMarkX, firstSpikeMarkY, FIRST_SPIKE_MARK_HEADING);

        // Construct a second spike mark pose.
        Pose2d secondSpikeMarkPose = new Pose2d(secondSpikeMarkX, secondSpikeMarkY, SECOND_SPIKE_MARK_HEADING);

        // Construct a third spike mark pose.
        Pose2d thirdSpikeMarkPose = new Pose2d(thirdSpikeMarkX, thirdSpikeMarkY, THIRD_SPIKE_MARK_HEADING);

        // Construct a first human pose.
        Pose2d firstHumanPose = new Pose2d(HUMAN_X, HUMAN_Y, FIRST_HUMAN_HEADING);

        // Construct a second human pose.
        Pose2d secondHumanPose = new Pose2d(HUMAN_X, HUMAN_Y, SECOND_HUMAN_HEADING);

        // Construct a rungs pose.
        Pose2d rungsPose = new Pose2d(RUNGS_X, RUNGS_Y, RUNGS_HEADING);

        // Construct a chamber pose.
        Pose2d chamberPose = new Pose2d(CHAMBER_X, CHAMBER_Y, CHAMBER_HEADING);

        // Construct an action for driving from the start to the basket.
        Action driveFromStartToBasket = drive.actionBuilder(startPose)
                .strafeToLinearHeading(basketPose.position, basketPose.heading)
                .build();

        // Construct an action for driving from the basket to the first spike mark.
        Action driveFromBasketToFirstSpikeMark = drive.actionBuilder(basketPose)
                .strafeToLinearHeading(firstSpikeMarkPose.position, firstSpikeMarkPose.heading, spikeMarkVelocityConstraint)
                .build();

        // Construct an action for driving from the first spike mark to the basket.
        Action driveFromFirstSpikeMarkToBasket = drive.actionBuilder(firstSpikeMarkPose)
                .strafeToLinearHeading(basketPose.position, basketPose.heading)
                .build();

        // Construct an action for driving from the first and a half spike mark to the second spike mark.
        Action driveFromBasketToSecondSpikeMark = drive.actionBuilder(basketPose)
                .strafeToLinearHeading(secondSpikeMarkPose.position, secondSpikeMarkPose.heading, spikeMarkVelocityConstraint)
                .build();

        // Construct an action for driving from the second spike mark to the basket.
        Action driveFromSecondSpikeMarkToBasket = drive.actionBuilder(secondSpikeMarkPose)
                .strafeToLinearHeading(basketPose.position, basketPose.heading)
                .build();

        // Construct an action for driving from the basket to the third spike mark.
        Action driveFromBasketToThirdSpikeMark = drive.actionBuilder(basketPose)
                .strafeToLinearHeading(thirdSpikeMarkPose.position, thirdSpikeMarkPose.heading, spikeMarkVelocityConstraint)
                .build();

        // Construct an action for driving from the third spike mark to the basket.
        Action driveFromThirdSpikeMarkToBasket = drive.actionBuilder(thirdSpikeMarkPose)
                .strafeToLinearHeading(basketPose.position, basketPose.heading)
                .build();

        // Construct an action for driving from the basket to the human.
        Action driveFromBasketToHuman = drive.actionBuilder(basketPose)
                .setReversed(true)
                .splineTo(firstHumanPose.position, firstHumanPose.heading, humanSampleVelocityConstraint)
                .build();

        // Construct an action for driving from the human to the basket.
        Action driveFromHumanToBasket = drive.actionBuilder(secondHumanPose)
                .splineTo(basketPose.position, basketPose.heading, humanVelocityConstraint)
                .build();

        // Construct an action for driving from the basket to the rungs.
        Action driveFromBasketToRungs = drive.actionBuilder(basketPose)
                .setReversed(true)
                .splineTo(rungsPose.position, rungsPose.heading)
                .build();

        // Construct an action for driving from the basket to the chamber.
        Action driveFromBasketToChamber = drive.actionBuilder(basketPose)
                .setReversed(true)
                .splineTo(chamberPose.position, chamberPose.heading)
                .build();

        // Construct a main action.
        Action mainAction = new SequentialAction(

                // Drive from the start position to the basket and move the arm so it does not hit the basket when raising.
                new ParallelAction(

                        // Drive from the start position to the basket.
                        driveFromStartToBasket,

                        // Raise the preloaded sample to the basket.
                        robotHardware.raiseSampleToBasket()

                ),

                // If appropriate, score the preloaded sample and then get the human sample.
                grabHumanSample == GrabHumanSample.BEGINNING ?
                        scoreAndGrab(robotHardware, driveFromBasketToHuman, driveFromHumanToBasket, SampleType.HUMAN) :
                        new SequentialAction(),

                // Score the current sample and then get the first spike mark sample.
                scoreAndGrab(robotHardware, driveFromBasketToFirstSpikeMark, driveFromFirstSpikeMarkToBasket, SampleType.NORMAL),

                // Score the first spike mark sample and then get the second spike mark sample.
                scoreAndGrab(robotHardware, driveFromBasketToSecondSpikeMark, driveFromSecondSpikeMarkToBasket, SampleType.NORMAL),

                // Score the second spike mark sample and then get the third spike mark sample.
                scoreAndGrab(robotHardware, driveFromBasketToThirdSpikeMark, driveFromThirdSpikeMarkToBasket, SampleType.WALL),

                // If appropriate, score the third spike mark sample and then get the human sample.
                grabHumanSample == GrabHumanSample.END ?
                    scoreAndGrab(robotHardware, driveFromBasketToHuman, driveFromHumanToBasket, SampleType.HUMAN) :
                    new SequentialAction(),

                // Score the last sample.
                robotHardware.scoreSample(),

                // Lower the arm and park.
                new ParallelAction(

                        // Lower the arm from the basket.
                        robotHardware.lowerArmFromBasket(true, true, false, false),

                        // Park.
                        rungsPark ? driveFromBasketToRungs : driveFromBasketToChamber

                )

        );

        // Return the main action.
        return mainAction;

    }

    // Adds telemetry.
    private void addTelemetry() {

        // Get a banner.
        String banner = getBanner(YELLOW_SQUARE);

        // Get the tall walls value.
        boolean tallWalls = robotHardware.getTallWalls();

        // Construct a walls string.
        String wallsString = tallWalls ? "Tall" : "Short";

        // Construct a park string.
        String parkString = rungsPark ? "Rungs" : "Chamber";

        // Display main telemetry.
        telemetry.addData("AutoSample", banner);
        telemetry.addData("- Grab Human Sample", grabHumanSample);
        telemetry.addData("- Park", parkString);
        telemetry.addData("- Walls", wallsString);

    }

}