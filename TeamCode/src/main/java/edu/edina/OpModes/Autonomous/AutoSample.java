package edu.edina.OpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.Robot.CloseClaw;
import edu.edina.Libraries.Robot.MoveToGround;
import edu.edina.Libraries.Robot.MoveToHighBasket;
import edu.edina.Libraries.Robot.OpenClaw;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.WaitAndUpdate;
import edu.edina.Libraries.Robot.WaitForNotBusy;

@Config
@Autonomous(preselectTeleOp = "TeleOpMain")
public class AutoSample extends LinearOpMode {

    // Start pose
    public static double START_X = -38;
    public static double START_Y = -62;
    public static double START_HEADING = 0;

    // Basket pose
    public static double BASKET_X = -50;
    public static double BASKET_Y = -50;
    public static double BASKET_HEADING = 1.0 / 4 * Math.PI;

    // First spike mark pose
    public static double FIRST_SPIKE_MARK_X = -49;
    public static double FIRST_SPIKE_MARK_Y = -36.5;
    public static double FIRST_SPIKE_MARK_HEADING = 1.0 / 2 * Math.PI;

    // Second spike mark pose
    public static double SECOND_SPIKE_MARK_X = -61;
    public static double SECOND_SPIKE_MARK_Y = FIRST_SPIKE_MARK_Y;
    public static double SECOND_SPIKE_MARK_HEADING = FIRST_SPIKE_MARK_HEADING;

    // First and a half spike mark pose
    public static double FIRST_AND_A_HALF_SPIKE_MARK_X = SECOND_SPIKE_MARK_X;
    public static double FIRST_AND_A_HALF_SPIKE_MARK_Y = -44;
    public static double FIRST_AND_A_HALF_SPIKE_MARK_HEADING = FIRST_SPIKE_MARK_HEADING;

    // Third spike mark pose
    public static double THIRD_SPIKE_MARK_X = -58;
    public static double THIRD_SPIKE_MARK_Y = -28;
    public static double THIRD_SPIKE_MARK_HEADING = Math.PI;

    // Duration in milliseconds to toggle the claw
    public static int CLAW_DELAY = 500;

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

        // Run the actions.
        Actions.runBlocking(
                new SequentialAction(

                        // Score preloaded sample.
                        driveFromStartToBasket,
                        score(),

                        // Score first spike mark sample.
                        driveFromBasketToFirstSpikeMark,
                        new CloseClaw(robotHardware),
                        new WaitAndUpdate(robotHardware, CLAW_DELAY, true),
                        driveFromFirstSpikeMarkToBasket,
                        score(),

                        // Score second spike mark sample.
                        driveFromBasketToFirstAndAHalfSpikeMark,
                        driveFromFirstAndAHalfToSecondSpikeMark,
                        new CloseClaw(robotHardware),
                        new WaitAndUpdate(robotHardware, CLAW_DELAY, true),
                        driveFromSecondSpikeMarkToBasket,
                        score(),

                        // Score second third mark sample.
                        driveFromBasketToThirdSpikeMark,
                        new CloseClaw(robotHardware),
                        new WaitAndUpdate(robotHardware, CLAW_DELAY, true),
                        driveFromThirdSpikeMarkToBasket,
                        score()

                )

        );

    }

    // Scores a sample.
    public Action score() {
        return new SequentialAction(
                new MoveToHighBasket(robotHardware),
                new WaitForNotBusy(robotHardware, true),
                new OpenClaw(robotHardware),
                new WaitAndUpdate(robotHardware, CLAW_DELAY, true),
                new MoveToGround(robotHardware),
                new WaitForNotBusy(robotHardware, true)
        );
    }

}