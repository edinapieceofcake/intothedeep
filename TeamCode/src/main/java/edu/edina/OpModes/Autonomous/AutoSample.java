package edu.edina.OpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.Robot.Arm;
import edu.edina.Libraries.Robot.MoveArm;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.WaitForTime;
import edu.edina.Libraries.Robot.WaitForHardware;

@Config
@Autonomous(preselectTeleOp = "TeleOpMain")
public class AutoSample extends LinearOpMode {

    // Start pose
    public static double START_X = -38;
    public static double START_Y = -61;
    public static double START_HEADING = 0;

    // Basket pose
    public static double BASKET_X = -49;
    public static double BASKET_Y = -49;
    public static double BASKET_HEADING = 1.0 / 4 * Math.PI;

    // First spike mark pose
    public static double FIRST_SPIKE_MARK_X = -49;
    public static double FIRST_SPIKE_MARK_Y = -36.5;
    public static double FIRST_SPIKE_MARK_HEADING = 1.0 / 2 * Math.PI;

    // Second spike mark pose
    public static double SECOND_SPIKE_MARK_X = -60;
    public static double SECOND_SPIKE_MARK_Y = FIRST_SPIKE_MARK_Y;
    public static double SECOND_SPIKE_MARK_HEADING = FIRST_SPIKE_MARK_HEADING;

    // First and a half spike mark pose
    public static double FIRST_AND_A_HALF_SPIKE_MARK_X = SECOND_SPIKE_MARK_X;
    public static double FIRST_AND_A_HALF_SPIKE_MARK_Y = -44;
    public static double FIRST_AND_A_HALF_SPIKE_MARK_HEADING = FIRST_SPIKE_MARK_HEADING;

    // Third spike mark pose
    public static double THIRD_SPIKE_MARK_X = -57;
    public static double THIRD_SPIKE_MARK_Y = -28;
    public static double THIRD_SPIKE_MARK_HEADING = Math.PI;

    // Duration in milliseconds to toggle the claw
    public static int CLAW_MILLISECONDS = 500;

    // Timeout in milliseconds
    public static int TIMEOUT_MILLISECONDS = 3000;

    // Robot hardware
    private RobotHardware robotHardware;

    // Runs the op mode.
    @Override
    public void runOpMode() throws InterruptedException {

        // Get hardware.
        robotHardware = new RobotHardware(this);

        // Wait for the user to lower the lift.
        robotHardware.waitForLiftDown();

        // Wait for the user to lower the arm.
        robotHardware.waitForArmDown();

        // Move the arm to the ground position.
        robotHardware.setArmGroundPosition();

        // Close the claw.
        robotHardware.closeClaw();

        // Raise the wrist.
        robotHardware.initializeWrist();

        // If stop is requested...
        if (isStopRequested()) {

            // Exit the method.
            return;

        }

        // Prompt the user to press start.
        robotHardware.log("Waiting for start...");

        // Wait for the user to press start.
        waitForStart();

        // Indicate that this is running.
        robotHardware.log("Running...");

        // Construct a start pose.
        Pose2d startPose = new Pose2d(START_X, START_Y, START_HEADING);

        // Construct a basket pose.
        Pose2d basketPose = new Pose2d(BASKET_X, BASKET_Y, BASKET_HEADING);

        // Construct a first spike mark pose.
        Pose2d firstSpikeMarkPose = new Pose2d(FIRST_SPIKE_MARK_X, FIRST_SPIKE_MARK_Y, FIRST_SPIKE_MARK_HEADING);

        // Construct a first and a half spike mark pose.
        Pose2d firstAndAHalfSpikeMarkPose = new Pose2d(FIRST_AND_A_HALF_SPIKE_MARK_X, FIRST_AND_A_HALF_SPIKE_MARK_Y, FIRST_AND_A_HALF_SPIKE_MARK_HEADING);

        // Construct a second spike mark pose.
        Pose2d secondSpikeMarkPose = new Pose2d(SECOND_SPIKE_MARK_X, SECOND_SPIKE_MARK_Y, SECOND_SPIKE_MARK_HEADING);

        // Construct a third spike mark pose.
        Pose2d thirdSpikeMarkPose = new Pose2d(THIRD_SPIKE_MARK_X, THIRD_SPIKE_MARK_Y, THIRD_SPIKE_MARK_HEADING);

        // Construct a drive interface.
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Construct an action for driving from the start to the basket.
        Action driveFromStartToBasket = drive.actionBuilder(startPose)
                .strafeToLinearHeading(basketPose.position, basketPose.heading)
                .build();

        // Construct an action for driving from the basket to the first spike mark.
        Action driveFromBasketToFirstSpikeMark = drive.actionBuilder(basketPose)
                .strafeToLinearHeading(firstSpikeMarkPose.position, firstSpikeMarkPose.heading)
                .build();

        // Construct an action for driving from the first spike mark to the basket.
        Action driveFromFirstSpikeMarkToBasket = drive.actionBuilder(firstSpikeMarkPose)
                .strafeToLinearHeading(basketPose.position, basketPose.heading)
                .build();

        // Construct an action for driving from the basket to the first and half spike mark.
        Action driveFromBasketToFirstAndAHalfSpikeMark = drive.actionBuilder(basketPose)
                .strafeToLinearHeading(firstAndAHalfSpikeMarkPose.position, firstAndAHalfSpikeMarkPose.heading)
                .build();

        // Construct an action for driving from the first and a half spike mark to the second spike mark.
        Action driveFromFirstAndAHalfToSecondSpikeMark = drive.actionBuilder(firstAndAHalfSpikeMarkPose)
                .strafeToLinearHeading(secondSpikeMarkPose.position, secondSpikeMarkPose.heading)
                .build();

        // Construct an action for driving from the second spike mark to the basket.
        Action driveFromSecondSpikeMarkToBasket = drive.actionBuilder(secondSpikeMarkPose)
                .strafeToLinearHeading(basketPose.position, basketPose.heading)
                .build();

        // Construct an action for driving from the basket to the third spike mark.
        Action driveFromBasketToThirdSpikeMark = drive.actionBuilder(basketPose)
                .strafeToLinearHeading(thirdSpikeMarkPose.position, thirdSpikeMarkPose.heading)
                .build();

        // Construct an action for driving from the third spike mark to the basket.
        Action driveFromThirdSpikeMarkToBasket = drive.actionBuilder(thirdSpikeMarkPose)
                .strafeToLinearHeading(basketPose.position, basketPose.heading)
                .build();

        // Construct a main action.
        Action mainAction = new SequentialAction(

                // Lower the wrist.
                new InstantAction(() -> robotHardware.lowerWrist()),

                // Score preloaded sample.
                driveFromStartToBasket,
                raiseAndScoreSample(),

                // Score first spike mark sample.
                driveFromBasketToFirstSpikeMark,
                new InstantAction(() -> robotHardware.closeClaw()),
                new WaitForTime(CLAW_MILLISECONDS),
                driveFromFirstSpikeMarkToBasket,
                raiseAndScoreSample(),

                // Score second spike mark sample.
                driveFromBasketToFirstAndAHalfSpikeMark,
                driveFromFirstAndAHalfToSecondSpikeMark,
                new InstantAction(() -> robotHardware.closeClaw()),
                new WaitForTime(CLAW_MILLISECONDS),
                driveFromSecondSpikeMarkToBasket,
                raiseAndScoreSample(),

                // Score second third mark sample.
                driveFromBasketToThirdSpikeMark,
                new InstantAction(() -> robotHardware.closeClaw()),
                new WaitForTime(CLAW_MILLISECONDS),
                driveFromThirdSpikeMarkToBasket,
                raiseAndScoreSample(),

                // Wait for everything to finish.
                new WaitForTime(TIMEOUT_MILLISECONDS)

        );

        // Disable manual driving.
        robotHardware.disableManualDriving();

        // Add the action to the robot hardware.
        robotHardware.addAction(mainAction);

        // While the op mode is active...
        while (opModeIsActive()) {

            // Update the robot hardware.
            robotHardware.update();

            // Update the telemetry.
            telemetry.update();

        }

    }

    public static Action raiseSample(RobotHardware robotHardware) {

        // Construct a raise and score sample action.
        Action action = new ParallelAction(
                new MoveArm(robotHardware, Arm.HIGH_BASKET_POSITION, false),
                new InstantAction(() -> robotHardware.setHighBasketExtension()),
                new SequentialAction(
                        new WaitForTime(500),
                        new InstantAction(() -> robotHardware.setLiftHighBasketPosition())
                )
        );

        return action;

    }

    // Raises and scores a sample.
    public Action raiseAndScoreSample() {

        // Construct a raise and score sample action.
        Action action = new SequentialAction(
                raiseSample(robotHardware),
                new WaitForHardware(robotHardware, TIMEOUT_MILLISECONDS),
                scoreSample(robotHardware)
        );

        // Return the action.
        return action;

    }

    // Scores a sample.
    public static Action scoreSample(RobotHardware robotHardware) {

        // Construct a score sample action.
        Action action = new SequentialAction(
                new InstantAction(() -> robotHardware.openClaw()),
                new WaitForTime(CLAW_MILLISECONDS),
                new ParallelAction(
                        new SequentialAction(
                                new WaitForTime(500),
                                new InstantAction(() -> robotHardware.raiseWrist())
                        ),
                        new MoveArm(robotHardware, Arm.GROUND_POSITION, false),
                        new InstantAction(() -> robotHardware.setMinimumExtension()),
                        new InstantAction(() -> robotHardware.setLiftGroundPosition())
                ),
                new WaitForHardware(robotHardware, TIMEOUT_MILLISECONDS),
                new InstantAction(() -> robotHardware.lowerWrist())
        );

        // Return the action.
        return action;

    }

}