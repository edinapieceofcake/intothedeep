package edu.edina.OpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.Robot.Arm;
import edu.edina.Libraries.Robot.MoveArm;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.WaitForHardware;
import edu.edina.Libraries.Robot.WaitForTime;

@Config
@Autonomous(preselectTeleOp = "TeleOpMain")
public class AutoSample extends LinearOpMode {

    // Last pose
    public static Pose2d lastPose;

    // Start pose
    public static double START_X = -35;
    public static double START_Y = -61;
    public static double START_HEADING = Math.toRadians(180);

    // Basket pose
    public static double BASKET_X = -58;
    public static double BASKET_Y = -58;
    public static double BASKET_HEADING = Math.toRadians(225);
    public static double BASKET_TANGENT = Math.toRadians(225);

    // First spike mark pose
    public static double FIRST_SPIKE_MARK_X = -48;
    public static double FIRST_SPIKE_MARK_Y = -38;
    public static double FIRST_SPIKE_MARK_HEADING = Math.toRadians(270);

    // Second spike mark pose
    public static double SECOND_SPIKE_MARK_X = -59;
    public static double SECOND_SPIKE_MARK_Y = FIRST_SPIKE_MARK_Y;
    public static double SECOND_SPIKE_MARK_HEADING = FIRST_SPIKE_MARK_HEADING;
    //public static double SECOND_SPIKE_MARK_BEGIN_TANGENT = 180;
    //public static double SECOND_SPIKE_MARK_END_TANGENT = 90;

    // Third spike mark pose
    public static double THIRD_SPIKE_MARK_X = -54;
    public static double THIRD_SPIKE_MARK_Y = -25;
    public static double THIRD_SPIKE_MARK_HEADING = Math.toRadians(0);
    /*
    // Human player pose a
    public static double HUMAN_PLAYER_A_X = -20;
    public static double HUMAN_PLAYER_A_Y = -60;
    public static double HUMAN_PLAYER_A_HEADING = Math.toRadians(0);
    public static double HUMAN_PLAYER_A_START_TANGENT = Math.toRadians(0);
    public static double HUMAN_PLAYER_A_END_TANGENT = HUMAN_PLAYER_A_START_TANGENT;

    // Human player pose b
    public static double HUMAN_PLAYER_B_X = 32;
    public static double HUMAN_PLAYER_B_Y = HUMAN_PLAYER_A_Y;
    public static double HUMAN_PLAYER_B_HEADING = HUMAN_PLAYER_A_HEADING;
    public static double HUMAN_PLAYER_B_START_TANGENT = HUMAN_PLAYER_A_START_TANGENT;
    public static double HUMAN_PLAYER_B_END_TANGENT = HUMAN_PLAYER_A_START_TANGENT;

    // Going back from human player tangent and heading
    public static double HUMAN_PLAYER_BACK_HEADING_AND_TANGENT = Math.toRadians(180);
    */
    // Duration in milliseconds to toggle the claw
    public static int CLAW_MILLISECONDS = 500;

    // Timeout in milliseconds
    public static int TIMEOUT_MILLISECONDS = 3500;

    // Fast velocity
    public static double FAST_VELOCITY = 30;

    // Slow velocity
    public static double SLOW_VELOCITY = 15;

    // Robot hardware
    private RobotHardware robotHardware;

    // Runs the op mode.
    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the robot.
        //////////////////////////////////////////////////////////////////////

        // Get hardware.
        robotHardware = new RobotHardware(this);

        // Wait for the user to lower the lift.
        robotHardware.waitForLiftDown();

        // Wait for the user to lower the arm.
        robotHardware.waitForArmDown();

        // Get the use big claw value.
        boolean useBigClaw = robotHardware.getUseBigClaw();

        // If we are using the big claw...
        if(useBigClaw) {

            // Close the big claw.
            robotHardware.closeBigClaw();

            // Open the small claw.
            robotHardware.openSmallClaw();

        }

        // Otherwise (if we are using the small claw)...
        else {

            // Open the big claw.
            robotHardware.openBigClaw();

            // Close the small claw.
            robotHardware.closeSmallClaw();

        }

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

            // Update the robot hardware.
            robotHardware.update();

            // Update the telemetry.
            telemetry.update();

            // Update the last pose.
            lastPose = robotHardware.drive.pose;

        }

    }

    // Scores the current sample and then gets a spike mark sample.
    private static Action scoreCurrentSampleAndThenGetSpikeMarkSample(RobotHardware robotHardware, Action driveFromBasketToSpikeMark, Action driveFromSpikeMarkToBasket, boolean horizontalSwivel) {

        // Get the use big claw value.
        boolean useBigClaw = robotHardware.getUseBigClaw();

        // Construct an action.
        Action action = new SequentialAction(

                // Score the current sample.
                robotHardware.scoreSample(),

                // Lower the arm while driving to the spike mark.
                new ParallelAction(

                        // Lower the arm.
                        new SequentialAction(
                                robotHardware.lowerArmFromBasket(horizontalSwivel, true),
                                new MoveArm(robotHardware, Arm.SUBMERSIBLE_GRAB_POSITION, false),
                                new WaitForHardware(robotHardware, 1000)
                        ),

                        // Drive to the spike mark.
                        driveFromBasketToSpikeMark

                ),

                // Wait for the arm to settle.
                new WaitForTime(500),

                // Grab the spike mark sample.
                useBigClaw ?
                        new InstantAction(() -> robotHardware.closeBigClaw()) :
                        new InstantAction(() -> robotHardware.closeSmallClaw()),
                new WaitForTime(CLAW_MILLISECONDS),

                // Drive to the basket and raise the sample.
                new ParallelAction(

                        // Drive to the basket.
                        driveFromSpikeMarkToBasket,

                        // Raise the sample to the basket.
                        robotHardware.raiseSampleToBasket()

                )

        );

        // Return the action.
        return action;

    }

    /*
    private static Action getHumanPlayerAction(RobotHardware robotHardware, Action driveFromBasketToHumanPlayer, Action driveFromHumanPlayerToBasket) {
        // Score human player sample.
        Action action = new SequentialAction(
                new ParallelAction(
                        robotHardware.scoreSampleTeleop(),
                        driveFromBasketToHumanPlayer
                ),
                new InstantAction(() -> robotHardware.closeClaw()),
                new WaitForTime(CLAW_MILLISECONDS),
                new ParallelAction(
                        driveFromHumanPlayerToBasket,
                        robotHardware.raiseSample()
                )
        );
        return action;
    }
    */
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
                return FAST_VELOCITY;

            }

        };

        // Construct trajectories.
        //////////////////////////////////////////////////////////////////////

        // Construct a start pose.
        Pose2d startPose = new Pose2d(START_X, START_Y, START_HEADING);

        // Construct a basket pose.
        Pose2d basketPose = new Pose2d(BASKET_X, BASKET_Y, BASKET_HEADING);

        // Construct a first spike mark pose.
        Pose2d firstSpikeMarkPose = new Pose2d(FIRST_SPIKE_MARK_X, FIRST_SPIKE_MARK_Y, FIRST_SPIKE_MARK_HEADING);

        // Construct a second spike mark pose.
        Pose2d secondSpikeMarkPose = new Pose2d(SECOND_SPIKE_MARK_X, SECOND_SPIKE_MARK_Y, SECOND_SPIKE_MARK_HEADING);

        // Construct a third spike mark pose.
        Pose2d thirdSpikeMarkPose = new Pose2d(THIRD_SPIKE_MARK_X, THIRD_SPIKE_MARK_Y, THIRD_SPIKE_MARK_HEADING);
        /*
        // Construct a human player pose a.
        Pose2d humanPlayerPoseA = new Pose2d(HUMAN_PLAYER_A_X, HUMAN_PLAYER_A_Y, HUMAN_PLAYER_A_HEADING);

        // Construct a human player pose b.
        Pose2d humanPlayerPoseB = new Pose2d(HUMAN_PLAYER_B_X, HUMAN_PLAYER_B_Y, HUMAN_PLAYER_B_HEADING);
        */
        // Construct a drive interface.
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

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
        /*
        // Construct an action for driving from the basket to the human player.
        Action driveFromBasketToHumanPlayer = drive.actionBuilder(basketPose)
                .setTangent(HUMAN_PLAYER_A_START_TANGENT)
                .splineToSplineHeading((humanPlayerPoseA), HUMAN_PLAYER_A_END_TANGENT)
                .setTangent(HUMAN_PLAYER_B_START_TANGENT)
                .splineToConstantHeading(new Vector2d(humanPlayerPoseB.position.x, humanPlayerPoseB.position.y), Math.toRadians(HUMAN_PLAYER_B_END_TANGENT))
                .build();

        // Construct an action for driving from the human player to the basket.
        Action driveFromHumanPlayerToBasket = drive.actionBuilder(humanPlayerPoseB)
                .setTangent(HUMAN_PLAYER_BACK_HEADING_AND_TANGENT)
                .splineToConstantHeading(new Vector2d(humanPlayerPoseA.position.x, humanPlayerPoseA.position.y), HUMAN_PLAYER_BACK_HEADING_AND_TANGENT)
                .setTangent(HUMAN_PLAYER_BACK_HEADING_AND_TANGENT)
                .splineToSplineHeading(basketPose, basketPose.heading)
                .build();
        */
        // Construct a main action.
        Action mainAction = new SequentialAction(

                // Drive from the start position to the basket and move the arm so it does not hit the basket when raising.
                new ParallelAction(
                        driveFromStartToBasket,
                        new MoveArm(robotHardware, Arm.BASKET_POSITION, true)
                ),

                // Raise the preloaded sample to the basket.
                robotHardware.raiseSampleToBasket(),

                // Score the preloaded sample and then get the first spike mark sample.
                scoreCurrentSampleAndThenGetSpikeMarkSample(robotHardware, driveFromBasketToFirstSpikeMark, driveFromFirstSpikeMarkToBasket, true),

                // Score the first spike mark sample and then get the second spike mark sample.
                scoreCurrentSampleAndThenGetSpikeMarkSample(robotHardware, driveFromBasketToSecondSpikeMark, driveFromSecondSpikeMarkToBasket, true),

                // Score the second spike mark sample and then get the third spike mark sample.
                scoreCurrentSampleAndThenGetSpikeMarkSample(robotHardware, driveFromBasketToThirdSpikeMark, driveFromThirdSpikeMarkToBasket, false),

                // Score the third spike mark sample.
                robotHardware.scoreSample(),

                // Lower the arm from the basket.
                robotHardware.lowerArmFromBasket(true, true)

        );

        // Return the main action.
        return mainAction;

    }

    // Determines whether the robot is close a spike mark.
    private static boolean isCloseToSpikeMark(Pose2dDual robotPose) {
        return robotPose.position.y.value() > -40;
    }

}