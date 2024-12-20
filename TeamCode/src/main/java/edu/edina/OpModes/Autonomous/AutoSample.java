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

import java.util.Stack;

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
    public static double BASKET_X = -50;
    public static double BASKET_Y = -50;
    public static double BASKET_HEADING = Math.toRadians(45);

    // First spike mark pose
    public static double FIRST_SPIKE_MARK_X = -48;
    public static double FIRST_SPIKE_MARK_Y = -33;
    public static double FIRST_SPIKE_MARK_HEADING = Math.toRadians(90);

    // Second spike mark pose
    public static double SECOND_SPIKE_MARK_X = -58.5;
    public static double SECOND_SPIKE_MARK_Y = FIRST_SPIKE_MARK_Y;
    public static double SECOND_SPIKE_MARK_HEADING = FIRST_SPIKE_MARK_HEADING;
    public static double SECOND_SPIKE_MARK_BEGIN_TANGENT = 180;
    public static double SECOND_SPIKE_MARK_END_TANGENT = 90;


    // First and a half spike mark pose
    public static double FIRST_AND_A_HALF_SPIKE_MARK_X = SECOND_SPIKE_MARK_X;
    public static double FIRST_AND_A_HALF_SPIKE_MARK_Y = -44;
    public static double FIRST_AND_A_HALF_SPIKE_MARK_HEADING = FIRST_SPIKE_MARK_HEADING;

    // Third spike mark pose
    public static double THIRD_SPIKE_MARK_X = -60;
    public static double THIRD_SPIKE_MARK_Y = -26;
    public static double THIRD_SPIKE_MARK_HEADING = Math.toRadians(180);

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

        // Move the arm to the ground position.
        robotHardware.setArmGroundPosition();

        // Close the claw.
        robotHardware.closeClaw();

        // Lower the wrist.
        robotHardware.lowerWrist();

        // Resets the swivel.
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

        // Get the main action.
        Action mainAction = getMainAction();

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
                .strafeToLinearHeading(firstSpikeMarkPose.position, firstSpikeMarkPose.heading, spikeMarkVelocityConstraint)
                .build();

        // Construct an action for driving from the first spike mark to the basket.
        Action driveFromFirstSpikeMarkToBasket = drive.actionBuilder(firstSpikeMarkPose)
                .strafeToLinearHeading(basketPose.position, basketPose.heading)
                .build();

        // Construct an action for driving from the basket to the first and half spike mark.
        Action driveFromBasketToFirstAndAHalfSpikeMark = drive.actionBuilder(basketPose)
                .strafeToLinearHeading(firstAndAHalfSpikeMarkPose.position, firstAndAHalfSpikeMarkPose.heading)
                .build();

        // Construct an action for driving from the basket to the second spike mark.
        Action driveFromBasketToSecondSpikeMark = drive.actionBuilder(basketPose)
                .setTangent(Math.toRadians(SECOND_SPIKE_MARK_BEGIN_TANGENT))
                .splineToLinearHeading(secondSpikeMarkPose, Math.toRadians(SECOND_SPIKE_MARK_END_TANGENT))
                .build();

        // Construct an action for driving from the first and a half spike mark to the second spike mark.
        Action driveFromFirstAndAHalfToSecondSpikeMark = drive.actionBuilder(firstAndAHalfSpikeMarkPose)
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

        // Construct a main action.
        //////////////////////////////////////////////////////////////////////

        // Construct a main action.
        Action mainAction = new SequentialAction(

                // Score preloaded sample.
                new ParallelAction(
                        driveFromStartToBasket,
                        raiseSampleAuto(robotHardware)
                ),

                // Score first spike mark sample.
                new ParallelAction(
                        scoreSample(robotHardware),
                        driveFromBasketToFirstSpikeMark
                ),
                new InstantAction(() -> robotHardware.closeClaw()),
                new WaitForTime(CLAW_MILLISECONDS),
                new ParallelAction(
                        driveFromFirstSpikeMarkToBasket,
                        raiseSampleAuto(robotHardware)
                ),

                // Score second spike mark sample.
                new ParallelAction(
                        scoreSample(robotHardware),
                        driveFromBasketToSecondSpikeMark
                ),
                new InstantAction(() -> robotHardware.closeClaw()),
                new WaitForTime(CLAW_MILLISECONDS),
                new ParallelAction(
                        driveFromSecondSpikeMarkToBasket,
                        raiseSampleAuto(robotHardware)
                ),

                // Score third spike mark sample.
                new ParallelAction(
                        scoreSample(robotHardware),
                        new InstantAction(() -> robotHardware.toggleSwivel()),
                        driveFromBasketToThirdSpikeMark
                ),
                new InstantAction(() -> robotHardware.closeClaw()),
                new WaitForTime(CLAW_MILLISECONDS),
                new ParallelAction(
                        driveFromThirdSpikeMarkToBasket,
                        raiseSampleAuto(robotHardware)
                ),
                scoreSample(robotHardware),

                // Wait for everything to finish.
                new WaitForTime(TIMEOUT_MILLISECONDS)

        );

        // Return the main action.
        return mainAction;

    }

    // Raises a sample in tele op.
    public static Action raiseSampleTeleOp(RobotHardware robotHardware) {

        // Construct a raise and score sample action.
        Action action = new ParallelAction(
                new MoveArm(robotHardware, Arm.HIGH_BASKET_POSITION, false),
                new SequentialAction(
                        new WaitForTime(500),
                        new InstantAction(() -> robotHardware.raiseWrist())
                ),
                new SequentialAction(
                        new WaitForTime(500),
                        new InstantAction(() -> robotHardware.setWristHighBasketPosition()),
                        new InstantAction(() -> robotHardware.setHighBasketExtension()),
                        new InstantAction(() -> robotHardware.setLiftHighBasketPosition())
                )
        );

        return action;

    }

    // Raises a sample in auto.
    public static Action raiseSampleAuto(RobotHardware robotHardware) {

        // Construct a raise and score sample action.
        Action action =
            new SequentialAction(
                new ParallelAction(
                    new MoveArm(robotHardware, Arm.HIGH_BASKET_POSITION, false),
                    new InstantAction(() -> robotHardware.setHighBasketExtension()),
                    new SequentialAction(
                            new WaitForTime(500),
                            new InstantAction(() -> robotHardware.setLiftHighBasketPosition())
                    )
                ),
                new InstantAction(() -> robotHardware.setWristHighBasketPosition())
            );

        return action;

    }

    // Raises and scores a sample.
    public Action raiseAndScoreSample() {

        // Construct a raise and score sample action.
        Action action = new SequentialAction(
                raiseSampleAuto(robotHardware),
                new WaitForHardware(robotHardware, TIMEOUT_MILLISECONDS),
                new WaitForTime(500),
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
            new WaitForHardware(robotHardware, TIMEOUT_MILLISECONDS)
        );

        // Return the action.
        return action;

    }

    // Determines whether the robot is close a spike mark.
    private static boolean isCloseToSpikeMark(Pose2dDual robotPose) {
        return robotPose.position.y.value() > -40;
    }

}