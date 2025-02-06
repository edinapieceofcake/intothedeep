package edu.edina.OpModes.Autonomous;

import static edu.edina.Libraries.Robot.RobotHardware.BLUE_SQUARE;
import static edu.edina.Libraries.Robot.RobotHardware.getBanner;
import static edu.edina.Libraries.Robot.RobotHardware.prompt;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.Robot.Arm;
import edu.edina.Libraries.Robot.MoveArm;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.WaitForTime;

@Config
@Autonomous(preselectTeleOp = "TeleOpMain")
public class AutoSpecimen extends LinearOpMode {

    // Start pose
    public static double START_X = 3;
    public static double START_Y = -60;
    public static double START_HEADING = Math.toRadians(270);

    // Preload chamber pose
    public static double PRELOAD_CHAMBER_X = START_X;
    public static double PRELOAD_CHAMBER_Y = -29;
    public static double PRELOAD_CHAMBER_HEADING = START_HEADING;

    // First spike mark pose
    public static double FIRST_SPIKE_MARK_X = 46.5;
    public static double FIRST_SPIKE_MARK_Y = -39.5;
    public static double FIRST_SPIKE_MARK_HEADING = Math.toRadians(270);
    public static double FIRST_SPIKE_MARK_END_TANGENT = Math.toRadians(90);

    // Second spike mark pose
    public static double SECOND_SPIKE_MARK_X = 56.5;
    public static double SECOND_SPIKE_MARK_Y = FIRST_SPIKE_MARK_Y;
    public static double SECOND_SPIKE_MARK_HEADING = FIRST_SPIKE_MARK_HEADING;
    public static double SECOND_SPIKE_MARK_END_TANGENT = FIRST_SPIKE_MARK_END_TANGENT;

    // First drop pose
    public static double FIRST_DROP_X = FIRST_SPIKE_MARK_X;
    public static double FIRST_DROP_Y = -51;
    public static double FIRST_DROP_HEADING = FIRST_SPIKE_MARK_HEADING;

    // Second drop pose
    public static double SECOND_DROP_X = 40;
    public static double SECOND_DROP_Y = FIRST_DROP_Y;
    public static double SECOND_DROP_HEADING = FIRST_DROP_HEADING;

    // Pick up pose
    public static double PICK_UP_X = SECOND_DROP_X;
    public static double PICK_UP_Y = -62;
    public static double PICK_UP_HEADING = FIRST_DROP_HEADING;
    public static double PICK_UP_TANGENT = Math.toRadians(270);

    // Chamber values
    public static double CHAMBER_TANGENT = Math.toRadians(90);
    public static double CHAMBER_X_SEPARATION = 1;
    public static double CHAMBER_X_CLOSE = 15;

    // First chamber pose
    public static double FIRST_CHAMBER_X = PRELOAD_CHAMBER_X - CHAMBER_X_SEPARATION;
    public static double FIRST_CHAMBER_Y = PRELOAD_CHAMBER_Y;
    public static double FIRST_CHAMBER_HEADING = PRELOAD_CHAMBER_HEADING;

    // Second chamber pose
    public static double SECOND_CHAMBER_X = FIRST_CHAMBER_X - CHAMBER_X_SEPARATION;
    public static double SECOND_CHAMBER_Y = PRELOAD_CHAMBER_Y;
    public static double SECOND_CHAMBER_HEADING = PRELOAD_CHAMBER_HEADING;

    // Third chamber pose
    public static double THIRD_CHAMBER_X = SECOND_CHAMBER_X - CHAMBER_X_SEPARATION;
    public static double THIRD_CHAMBER_Y = PRELOAD_CHAMBER_Y;
    public static double THIRD_CHAMBER_HEADING = PRELOAD_CHAMBER_HEADING;

    // Velocities
    public static double FAST_VELOCITY = 38;
    public static double MEDIUM_VELOCITY = 26;

    // Robot hardware
    private RobotHardware robotHardware;

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

            // If the tall walls value is missing...
            if(inputTallWalls == null) {

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

        // Get the main action.
        Action mainAction = getMainAction(drive, startPose);

        // Add the main action to the robot hardware.
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

    // Gets the main action.
    private Action getMainAction(MecanumDrive drive, Pose2d startPose) {

        // Construct velocity constraints.
        //////////////////////////////////////////////////////////////////////

        // Construct a first spike mark velocity constraint.
        VelConstraint firstSpikeMarkVelocityConstraint = (robotPose, _path, _disp) ->
                robotPose.position.x.value() > 35 ? MEDIUM_VELOCITY : FAST_VELOCITY;

        // Construct a chamber velocity constraints.
        VelConstraint firstChamberVelocityConstraint = (robotPose, _path, _disp) ->
                robotPose.position.x.value() < FIRST_CHAMBER_X + CHAMBER_X_CLOSE ? MEDIUM_VELOCITY : FAST_VELOCITY;
        VelConstraint secondChamberVelocityConstraint = (robotPose, _path, _disp) ->
                robotPose.position.x.value() < SECOND_CHAMBER_X + CHAMBER_X_CLOSE ? MEDIUM_VELOCITY : FAST_VELOCITY;
        VelConstraint thirdChamberVelocityConstraint = (robotPose, _path, _disp) ->
                robotPose.position.x.value() < THIRD_CHAMBER_X + CHAMBER_X_CLOSE ? MEDIUM_VELOCITY : FAST_VELOCITY;

        // Construct a preload velocity constraint.
        TranslationalVelConstraint preloadVelocityConstraint =
                new TranslationalVelConstraint(MEDIUM_VELOCITY);

        // Construct a pick up velocity constraint.
        VelConstraint pickUpVelocityConstraint = (robotPose, _path, _disp) ->
                robotPose.position.y.value() < -50 ? MEDIUM_VELOCITY : FAST_VELOCITY;

        // Construct poses.
        //////////////////////////////////////////////////////////////////////

        // Construct a chamber pose.
        Pose2d chamberPose = new Pose2d(PRELOAD_CHAMBER_X, PRELOAD_CHAMBER_Y, PRELOAD_CHAMBER_HEADING);

        // Construct a first spike mark pose.
        Pose2d firstSpikeMarkPose = new Pose2d(FIRST_SPIKE_MARK_X, FIRST_SPIKE_MARK_Y, FIRST_SPIKE_MARK_HEADING);

        // Construct a first drop pose.
        Pose2d firstDropPose = new Pose2d(FIRST_DROP_X, FIRST_DROP_Y, FIRST_DROP_HEADING);

        // Construct a second spike mark pose.
        Pose2d secondSpikeMarkPose = new Pose2d(SECOND_SPIKE_MARK_X, SECOND_SPIKE_MARK_Y, SECOND_SPIKE_MARK_HEADING);

        // Construct a second drop pose.
        Pose2d secondDropPose = new Pose2d(SECOND_DROP_X, SECOND_DROP_Y, SECOND_DROP_HEADING);

        // Construct a pick up pose.
        Pose2d pickUpPose = new Pose2d(PICK_UP_X, PICK_UP_Y, PICK_UP_HEADING);

        // Construct a pick up pose two.
        Pose2d pickUpPoseTwo = new Pose2d(PICK_UP_X, PICK_UP_Y, PICK_UP_HEADING);

        // Construct a first chamber pose.
        Pose2d firstChamberPose = new Pose2d(FIRST_CHAMBER_X, FIRST_CHAMBER_Y, FIRST_CHAMBER_HEADING);

        // Construct a second chamber pose.
        Pose2d secondChamberPose = new Pose2d(SECOND_CHAMBER_X, SECOND_CHAMBER_Y, SECOND_CHAMBER_HEADING);

        // Construct a third chamber pose.
        Pose2d thirdChamberPose = new Pose2d(THIRD_CHAMBER_X, THIRD_CHAMBER_Y, THIRD_CHAMBER_HEADING);

        // Construct trajectories.
        //////////////////////////////////////////////////////////////////////

        // Construct a drive from start to chamber trajectory.
        TrajectoryActionBuilder driveFromStartToChamberBuilder = drive.actionBuilder(startPose)
                .strafeTo(chamberPose.position, preloadVelocityConstraint);
        Action driveFromStartToChamber = driveFromStartToChamberBuilder.build();

        // Construct a drive from chamber to first spike mark trajectory.
        TrajectoryActionBuilder driveFromChamberToFirstSpikeMarkBuilder = driveFromStartToChamberBuilder.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(firstSpikeMarkPose.position, FIRST_SPIKE_MARK_END_TANGENT, firstSpikeMarkVelocityConstraint);
        Action driveFromChamberToFirstSpikeMark = driveFromChamberToFirstSpikeMarkBuilder.build();

        // Construct a drive from first spike mark to first drop trajectory.
        TrajectoryActionBuilder driveFromFirstSpikeMarkToFirstDropBuilder = driveFromChamberToFirstSpikeMarkBuilder.endTrajectory().fresh()
                .strafeTo(firstDropPose.position);
        Action driveFromFirstSpikeMarkToFirstDrop = driveFromFirstSpikeMarkToFirstDropBuilder.build();

        // Construct a drive from first drop to second spike mark trajectory.
        TrajectoryActionBuilder driveFromFirstDropToSecondSpikeMarkBuilder = driveFromFirstSpikeMarkToFirstDropBuilder.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(secondSpikeMarkPose.position, SECOND_SPIKE_MARK_END_TANGENT);
        Action driveFromFirstDropToSecondSpikeMark = driveFromFirstDropToSecondSpikeMarkBuilder.build();

        // Construct a drive from second spike mark to second drop trajectory.
        TrajectoryActionBuilder driveFromSecondSpikeMarkToSecondDropBuilder = driveFromFirstDropToSecondSpikeMarkBuilder.endTrajectory().fresh()
                .strafeTo(secondDropPose.position);
        Action driveFromSecondSpikeMarkToSecondDrop = driveFromSecondSpikeMarkToSecondDropBuilder.build();

        // Construct a drive from second drop to pick up trajectory.
        TrajectoryActionBuilder driveFromSecondDropToPickUpBuilder = driveFromSecondSpikeMarkToSecondDropBuilder.endTrajectory().fresh()
                .strafeTo(pickUpPose.position);
        Action driveFromSecondDropToPickUp = driveFromSecondDropToPickUpBuilder.build();

        // Construct a drive from pick up to first chamber trajectory.
        TrajectoryActionBuilder driveFromPickUpToFirstChamberBuilder = driveFromSecondDropToPickUpBuilder.endTrajectory().fresh()
                .setTangent(CHAMBER_TANGENT)
				.splineToConstantHeading(firstChamberPose.position, CHAMBER_TANGENT, firstChamberVelocityConstraint);
        Action driveFromPickUpToFirstChamber = driveFromPickUpToFirstChamberBuilder.build();

        // Construct a drive from first chamber to pick up trajectory.
        TrajectoryActionBuilder driveFromFirstChamberToPickUpBuilder = driveFromPickUpToFirstChamberBuilder.endTrajectory().fresh()
                .setTangent(PICK_UP_TANGENT)
				.splineToConstantHeading(pickUpPose.position, PICK_UP_TANGENT, pickUpVelocityConstraint);
        Action driveFromFirstChamberToPickUp = driveFromFirstChamberToPickUpBuilder.build();

        // Construct a drive from pick up to second chamber trajectory.
        TrajectoryActionBuilder driveFromPickUpToSecondChamberBuilder = driveFromFirstChamberToPickUpBuilder.endTrajectory().fresh()
                .setTangent(CHAMBER_TANGENT)
                .splineToConstantHeading(secondChamberPose.position, CHAMBER_TANGENT, secondChamberVelocityConstraint);
        Action driveFromPickUpToSecondChamber = driveFromPickUpToSecondChamberBuilder.build();

        // Construct a drive from second chamber to pick up trajectory.
        TrajectoryActionBuilder driveFromSecondChamberToPickUpBuilder = driveFromPickUpToSecondChamberBuilder.endTrajectory().fresh()
                .setTangent(PICK_UP_TANGENT)
                .splineToConstantHeading(pickUpPoseTwo.position, PICK_UP_TANGENT, pickUpVelocityConstraint);
        Action driveFromSecondChamberToPickUp = driveFromSecondChamberToPickUpBuilder.build();

        // Construct a drive from pick up to third chamber trajectory.
        TrajectoryActionBuilder driveFromPickUpToThirdChamberBuilder = driveFromSecondChamberToPickUpBuilder.endTrajectory().fresh()
                .setTangent(CHAMBER_TANGENT)
                .splineToConstantHeading(thirdChamberPose.position, CHAMBER_TANGENT, thirdChamberVelocityConstraint);
        Action driveFromPickUpToThirdChamber = driveFromPickUpToThirdChamberBuilder.build();

        // Construct a drive nowhere trajectory.
        Action driveNowhere = new SequentialAction();

        // Construct a main action.
        //////////////////////////////////////////////////////////////////////

        // Get the tall walls value.
        boolean tallWalls = robotHardware.getTallWalls();

        // Get an wall arm position
        int wallArmPosition = tallWalls ? Arm.SUBMERSIBLE_TO_TALL_WALL_POSITION : Arm.SUBMERSIBLE_TO_SHORT_WALL_POSITION;

        // Construct a main action.
        Action mainAction = new SequentialAction(

                // Score the preloaded specimen and drive to the first spike mark.
                scoreAndDrive(driveFromStartToChamber, driveFromChamberToFirstSpikeMark, Arm.SUBMERSIBLE_ENTER_POSITION, true),

                // Grab the first spike mark sample.
                grabSample(),

                // Deliver the first spike mark sample to the human player.
                deliverSample(driveFromFirstSpikeMarkToFirstDrop),

                // Drive to the second spike mark.
                driveToSpikeMark(driveFromFirstDropToSecondSpikeMark, true),

                // Grab the second spike mark sample.
                grabSample(),

                // Deliver the second spike mark sample to the human player.
                deliverSample(driveFromSecondSpikeMarkToSecondDrop),

                // Drive to pick up.
                new InstantAction(() -> robotHardware.swivelSetHorizontal()),
                new ParallelAction(
                        new MoveArm(robotHardware, tallWalls ? Arm.GROUND_TO_TALL_WALL_POSITION : Arm.GROUND_TO_SHORT_WALL_POSITION, true),
                        driveFromSecondDropToPickUp
                ),
                new WaitForTime(250),

                // Pick up specimen
                pickUpSpecimen(),

                // Score the first specimen and drive to the wall.
                scoreAndDrive(driveFromPickUpToFirstChamber, driveFromFirstChamberToPickUp, wallArmPosition, false),

                // Pick up specimen
                pickUpSpecimen(),

                // Score the second specimen and drive to the wall.
                scoreAndDrive(driveFromPickUpToSecondChamber, driveFromSecondChamberToPickUp, wallArmPosition, false),

                // Pick up specimen
                pickUpSpecimen(),

                // Score the third specimen.
                scoreAndDrive(driveFromPickUpToThirdChamber, driveNowhere, Arm.MINIMUM_POSITION, false)

        );

        // Return the main action.
        return mainAction;

    }

    // Scores a specimen and then drives to a destination.
    private Action scoreAndDrive(Action driveToChamber, Action driveToDestination, int finalArmPosition, boolean isPreload) {

        // Construct an action.
        Action action = new SequentialAction(

                // Drive to the chamber while raising the specimen.
                new ParallelAction(
                        new SequentialAction(
                                new WaitForTime(250),
                                driveToChamber
                        ),
                        robotHardware.raiseToChamber()
                ),

                // Score the specimen.
                new InstantAction(() -> robotHardware.openSmallClaw()),
                new WaitForTime(100),
                new InstantAction(() -> robotHardware.setWristWallPosition()),

                // Drive to the destination while lowering the arm.
                new ParallelAction(
                        driveToDestination,
                        new SequentialAction(
                                new WaitForTime(isPreload ? 1000 : 800),
                                new ParallelAction(
                                        new InstantAction(() -> robotHardware.setLiftGroundPosition()),
                                        finalArmPosition == Arm.SUBMERSIBLE_ENTER_POSITION ?
                                                new InstantAction(() -> robotHardware.setAutoExtension()) :
                                                new InstantAction(() -> robotHardware.setMinimumExtension()),
                                        new InstantAction(() -> robotHardware.openBigClaw()),
                                        new InstantAction(() -> robotHardware.swivelSetHorizontal()),
                                        new MoveArm(robotHardware, finalArmPosition, true),
                                        finalArmPosition == Arm.SUBMERSIBLE_TO_TALL_WALL_POSITION || finalArmPosition == Arm.SUBMERSIBLE_TO_SHORT_WALL_POSITION ?
                                                new InstantAction(() -> robotHardware.setWristWallPosition()) :
                                                new InstantAction(() -> robotHardware.setWristSubmersiblePosition())
                                )
                        )
                ),

                // Wait for the arm to settle.
                new WaitForTime(100)

        );

        // Return the action.
        return action;

    }

    // Grabs a sample from a spike mark.
    private Action grabSample() {

        // Construct an action.
        Action action = new SequentialAction(
                new MoveArm(robotHardware, Arm.SUBMERSIBLE_GRAB_POSITION, true),
                new InstantAction(() -> robotHardware.closeBigClaw()),
                new WaitForTime(250)
        );

        // Return the action.
        return action;

    }

    // Delivers a sample to the human player.
    private Action deliverSample(Action driveToHuman) {

        // Construct an action.
        Action action = new SequentialAction(
                new ParallelAction(
                        new InstantAction(() -> robotHardware.setMinimumExtension()),
                        new MoveArm(robotHardware, Arm.GROUND_POSITION, true),
                        new InstantAction(() -> robotHardware.setWristWallPosition()),
                        driveToHuman
                ),
                new WaitForTime(200),
                new InstantAction(() -> robotHardware.openBigClaw()),
                new WaitForTime(200)
        );

        // Return the action.
        return action;

    }

    // Drives to a spike mark.
    private Action driveToSpikeMark(Action driveToSpikeMark, boolean horizontalSwivel) {

        // Construct an action.
        Action action = new SequentialAction(
                new ParallelAction(
                        horizontalSwivel ?
                                new SequentialAction(
                                        new InstantAction(() -> robotHardware.swivelSetHorizontal()),
                                        new InstantAction(() -> robotHardware.setAutoExtension()),
                                        new InstantAction(() -> robotHardware.setWristSubmersiblePosition()),
                                        new MoveArm(robotHardware, Arm.SUBMERSIBLE_ENTER_POSITION, true)
                                ) :
                                new SequentialAction(
                                        new InstantAction(() -> robotHardware.swivelSetVertical()),
                                        new InstantAction(() -> robotHardware.setMinimumExtension()),
                                        new InstantAction(() -> robotHardware.setWristSubmersiblePosition()),
                                        new MoveArm(robotHardware, Arm.SUBMERSIBLE_ENTER_POSITION, true),
                                        new InstantAction(() -> robotHardware.setAutoExtension())
                                ),
                        driveToSpikeMark
                ),
                new WaitForTime(200)
        );

        // Return the action.
        return action;

    }

    // Picks up a specimen from the wall.
    private Action pickUpSpecimen() {

        // Construct an action.
        Action action = new SequentialAction(
                new InstantAction(() -> robotHardware.closeBigClaw()),
                new WaitForTime(250)
        );

        // Return the action.
        return action;

    }

    // Adds telemetry.
    private void addTelemetry() {

        // Get a banner.
        String banner = getBanner(BLUE_SQUARE);

        // Get the tall walls value.
        boolean tallWalls = robotHardware.getTallWalls();

        // Construct a walls string.
        String walls = tallWalls ? "Tall" : "Short";

        // Display main telemetry.
        telemetry.addData("AutoSpecimen", banner);
        telemetry.addData("- Walls", walls);

    }

}