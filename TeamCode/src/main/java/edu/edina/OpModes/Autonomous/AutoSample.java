package edu.edina.OpModes.Autonomous;

import static edu.edina.Libraries.Robot.RobotHardware.YELLOW_SQUARE;
import static edu.edina.Libraries.Robot.RobotHardware.getBanner;
import static edu.edina.Libraries.Robot.RobotHardware.getSymbol;
import static edu.edina.Libraries.Robot.RobotHardware.prompt;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.Robot.Arm;
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

    // First basket pose
    public static double FIRST_BASKET_X = -58;
    public static double FIRST_BASKET_Y = -58;
    public static double FIRST_BASKET_HEADING = Math.toRadians(225);

    // Second basket pose
    public static double SECOND_BASKET_X = -58;
    public static double SECOND_BASKET_Y = -54;
    public static double SECOND_BASKET_HEADING = Math.toRadians(225);

    // First spike mark pose
    public static double FIRST_SPIKE_MARK_X = -48;
    public static double FIRST_SPIKE_MARK_Y = -37;
    public static double FIRST_SPIKE_MARK_HEADING = Math.toRadians(270);

    // Second spike mark pose
    public static double SECOND_SPIKE_MARK_X = -59;
    public static double SECOND_SPIKE_MARK_Y = -38;
    public static double SECOND_SPIKE_MARK_HEADING = FIRST_SPIKE_MARK_HEADING;

    // Third spike mark pose
    public static double THIRD_SPIKE_MARK_X = -56;
    public static double THIRD_SPIKE_MARK_Y = -25;
    public static double THIRD_SPIKE_MARK_HEADING = Math.toRadians(0);

    // Human pose
    public static double HUMAN_X = 34;
    public static double HUMAN_Y = -50.5;
    public static double FIRST_HUMAN_HEADING = Math.toRadians(0);
    public static double SECOND_HUMAN_HEADING = Math.toRadians(180);

    // Submersible pose
    public static double SUBMERSIBLE_X = -40;
    public static double SUBMERSIBLE_Y = -3;
    public static double SUBMERSIBLE_HEADING = Math.toRadians(0);

    // Timeout in milliseconds
    public static int TIMEOUT_MILLISECONDS = 3500;

    // Fast velocity
    public static double FAST_VELOCITY = 40;

    // Medium velocity
    public static double MEDIUM_VELOCITY = 30;

    // Slow velocity
    public static double SLOW_VELOCITY = 15;

    // Robot hardware
    private RobotHardware robotHardware;

    // Grab fifth sample value
    private Boolean grabFifthSample;

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

            // If the grab fifth sample value is missing...
            if(grabFifthSample == null) {

                // Prompt the user for a sample count.
                prompt(telemetry, "Samples", "X = 4, B = 5");

                // If the user pressed x...
                if (currentGamepad.x && !previousGamepad.x) {

                    // Do not grab a fifth sample.
                    grabFifthSample = false;

                }

                // If the user pressed b...
                if (currentGamepad.b && !previousGamepad.b) {

                    // Grab a fifth sample.
                    grabFifthSample = true;

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

                // If the user pressed b...
                if (currentGamepad.b && !previousGamepad.b) {

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

        // Indicate that the robot is initializing.
        robotHardware.log("Initializing...");

        // Initialize the robot.
        //////////////////////////////////////////////////////////////////////

        // Get hardware.
        robotHardware = new RobotHardware(this);

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

        // Set the main action.
        Action mainAction = getMainAction();

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
    private Action getMainAction() {

        // Construct velocity constraints.
        //////////////////////////////////////////////////////////////////////

        // Construct a spike mark velocity constraint.
        VelConstraint spikeMarkVelocityConstraint = (robotPose, _path, _disp) -> {

            // Determine whether the robot is close to a spike mark.
            boolean closeToSpikeMark = isCloseToSpikeMark(robotPose);

            // If the robot is close to a spike mark...
            if (closeToSpikeMark) {

                // Go slow.
                return SLOW_VELOCITY;

            }

            // Otherwise (if the robot is far from a spike mark)...
            else {

                // Go fast.
                return MEDIUM_VELOCITY;

            }

        };

        // Construct a submersible velocity constraint.
        VelConstraint submersibleVelocityConstraint = (robotPose, _path, _disp) -> {

            // Determine whether the robot is close to the submersible.
            boolean closeToSubmersible = isCloseToSubmersible(robotPose);

            // If the robot is close to the submersible...
            if (closeToSubmersible) {

                // Go slow.
                return MEDIUM_VELOCITY;

            }

            // Otherwise (if the robot is far from the submersible)...
            else {

                // Go fast.
                return FAST_VELOCITY;

            }

        };

        // Construct a fifth sample velocity constraint.
        VelConstraint fifthSampleVelocityConstraint = (robotPose, _path, _disp) -> {

            // Determine whether the robot is close to the fifth sample.
            boolean closeToFifthSample = isCloseToFifthSample(robotPose);

            // If the robot is close to the fifth sample...
            if (closeToFifthSample) {

                // Go slow.
                return SLOW_VELOCITY;

            }

            // Otherwise (if the robot is far from the fifth sample)...
            else {

                // Go fast.
                return FAST_VELOCITY;

            }

        };

        // Construct a human velocity constraint.
		TranslationalVelConstraint humanVelocityConstraint = new TranslationalVelConstraint(FAST_VELOCITY);

        // Construct trajectories.
        //////////////////////////////////////////////////////////////////////

        // Construct a start pose.
        Pose2d startPose = new Pose2d(START_X, START_Y, START_HEADING);

        // Construct a first basket pose.
        Pose2d firstBasketPose = new Pose2d(FIRST_BASKET_X, FIRST_BASKET_Y, FIRST_BASKET_HEADING);

        // Construct a second basket pose.
        Pose2d secondBasketPose = new Pose2d(SECOND_BASKET_X, SECOND_BASKET_Y, SECOND_BASKET_HEADING);

        // Construct a first spike mark pose.
        Pose2d firstSpikeMarkPose = new Pose2d(FIRST_SPIKE_MARK_X, FIRST_SPIKE_MARK_Y, FIRST_SPIKE_MARK_HEADING);

        // Construct a second spike mark pose.
        Pose2d secondSpikeMarkPose = new Pose2d(SECOND_SPIKE_MARK_X, SECOND_SPIKE_MARK_Y, SECOND_SPIKE_MARK_HEADING);

        // Construct a third spike mark pose.
        Pose2d thirdSpikeMarkPose = new Pose2d(THIRD_SPIKE_MARK_X, THIRD_SPIKE_MARK_Y, THIRD_SPIKE_MARK_HEADING);

        // Construct a first human pose.
        Pose2d firstHumanPose = new Pose2d(HUMAN_X, HUMAN_Y, FIRST_HUMAN_HEADING);

        // Construct a second human pose.
        Pose2d secondHumanPose = new Pose2d(HUMAN_X, HUMAN_Y, SECOND_HUMAN_HEADING);

        // Construct a submersible pose.
        Pose2d submersiblePose = new Pose2d(SUBMERSIBLE_X, SUBMERSIBLE_Y, SUBMERSIBLE_HEADING);

        // Construct a drive interface.
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Construct an action for driving from the start to the basket.
        Action driveFromStartToBasket = drive.actionBuilder(startPose)
                .strafeToLinearHeading(firstBasketPose.position, firstBasketPose.heading)
                .build();

        // Construct an action for driving from the basket to the first spike mark.
        Action driveFromBasketToFirstSpikeMark = drive.actionBuilder(firstBasketPose)
                .strafeToLinearHeading(firstSpikeMarkPose.position, firstSpikeMarkPose.heading, spikeMarkVelocityConstraint)
                .build();

        // Construct an action for driving from the first spike mark to the basket.
        Action driveFromFirstSpikeMarkToBasket = drive.actionBuilder(firstSpikeMarkPose)
                .strafeToLinearHeading(firstBasketPose.position, firstBasketPose.heading)
                .build();

        // Construct an action for driving from the first and a half spike mark to the second spike mark.
        Action driveFromBasketToSecondSpikeMark = drive.actionBuilder(firstBasketPose)
                .strafeToLinearHeading(secondSpikeMarkPose.position, secondSpikeMarkPose.heading, spikeMarkVelocityConstraint)
                .build();

        // Construct an action for driving from the second spike mark to the basket.
        Action driveFromSecondSpikeMarkToBasket = drive.actionBuilder(secondSpikeMarkPose)
                .strafeToLinearHeading(firstBasketPose.position, firstBasketPose.heading)
                .build();

        // Construct an action for driving from the basket to the third spike mark.
        Action driveFromBasketToThirdSpikeMark = drive.actionBuilder(firstBasketPose)
                .strafeToLinearHeading(thirdSpikeMarkPose.position, thirdSpikeMarkPose.heading, spikeMarkVelocityConstraint)
                .build();

        // Construct an action for driving from the third spike mark to the basket.
        Action driveFromThirdSpikeMarkToBasket = drive.actionBuilder(thirdSpikeMarkPose)
                .strafeToLinearHeading(firstBasketPose.position, firstBasketPose.heading)
                .build();

        // Construct an action for driving from the basket to the human.
        Action driveFromBasketToHuman = drive.actionBuilder(firstBasketPose)
                .setReversed(true)
                .splineTo(firstHumanPose.position, firstHumanPose.heading, fifthSampleVelocityConstraint)
                .build();

        // Construct an action for driving from the human to the basket.
        Action driveFromHumanToBasket = drive.actionBuilder(secondHumanPose)
                .splineTo(secondBasketPose.position, secondBasketPose.heading, humanVelocityConstraint)
                .build();

        // Construct an action for driving from the basket to the submersible
        Action driveFromBasketToSubmersible = drive.actionBuilder(secondBasketPose)
                .setReversed(true)
                .splineTo(submersiblePose.position, submersiblePose.heading, submersibleVelocityConstraint)
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

                // Score the preloaded sample and then get the first spike mark sample.
                scoreAndGrab(robotHardware, driveFromBasketToFirstSpikeMark, driveFromFirstSpikeMarkToBasket, SampleType.NORMAL),

                // Score the first spike mark sample and then get the second spike mark sample.
                scoreAndGrab(robotHardware, driveFromBasketToSecondSpikeMark, driveFromSecondSpikeMarkToBasket, SampleType.NORMAL),

                // Score the second spike mark sample and then get the third spike mark sample.
                scoreAndGrab(robotHardware, driveFromBasketToThirdSpikeMark, driveFromThirdSpikeMarkToBasket, SampleType.WALL),

                // If appropriate, score the third spike mark sample and then get the human sample.
                grabFifthSample ?
                    scoreAndGrab(robotHardware, driveFromBasketToHuman, driveFromHumanToBasket, SampleType.HUMAN) :
                    new SequentialAction(),

                // Score the last sample.
                robotHardware.scoreSample(),

                // Lower the arm and drive to the submersible.
                new ParallelAction(

                        // Lower the arm from the basket.
                        robotHardware.lowerArmFromBasket(true, true, false, false),

                        // Drive to the submersible.
                        driveFromBasketToSubmersible

                )

        );

        // Return the main action.
        return mainAction;

    }

    // Determines whether the robot is close to a spike mark.
    private static boolean isCloseToSpikeMark(Pose2dDual robotPose) {
        return robotPose.position.y.value() > -40;
    }

    // Determines whether the robot is close to the submersible.
    private static boolean isCloseToSubmersible(Pose2dDual robotPose) {
        return robotPose.position.y.value() > -25;
    }

    // Determines whether the robot is close to the fifth sample.
    private static boolean isCloseToFifthSample(Pose2dDual robotPose) {
        return robotPose.position.x.value() > 20;
    }

    // Adds telemetry.
    private void addTelemetry() {

        // Get a banner.
        String banner = getBanner(YELLOW_SQUARE);

        // Convert the grab fifth sample value to a symbol.
        String grabFifthSampleSymbol = getSymbol(grabFifthSample);

        // Get the tall walls value.
        boolean tallWalls = robotHardware.getTallWalls();

        // Convert the tall walls value to a symbol.
        String tallWallsSymbol = getSymbol(tallWalls);

        // Display main telemetry.
        telemetry.addData("AutoSample", banner);
        telemetry.addData("- Grab Fifth Sample", grabFifthSampleSymbol);
        telemetry.addData("- Tall Walls", tallWallsSymbol);

    }

}