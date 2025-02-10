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
    public static double FIRST_SPIKE_MARK_Y = -39;
    public static double FIRST_SPIKE_MARK_HEADING = Math.toRadians(270);
    public static double FIRST_SPIKE_MARK_END_TANGENT = Math.toRadians(90);

    // Second spike mark pose
    public static double SECOND_SPIKE_MARK_X = 56.5;
    public static double SECOND_SPIKE_MARK_Y = FIRST_SPIKE_MARK_Y;
    public static double SECOND_SPIKE_MARK_HEADING = FIRST_SPIKE_MARK_HEADING;
    public static double SECOND_SPIKE_MARK_END_TANGENT = FIRST_SPIKE_MARK_END_TANGENT;

    // Drop pose
    public static double DROP_X = FIRST_SPIKE_MARK_X;
    public static double DROP_Y = -51;
    public static double DROP_HEADING = FIRST_SPIKE_MARK_HEADING;

    // Approach pick up pose
    public static double APPROACH_PICK_UP_X = 40;
    public static double APPROACH_PICK_UP_Y = -47;
    public static double APPROACH_PICK_UP_HEADING = DROP_HEADING;
    public static double APPROACH_PICK_UP_TANGENT = Math.toRadians(270);

    // Pick up pose
    public static double PICK_UP_X = APPROACH_PICK_UP_X;
    public static double FIRST_PICK_UP_Y = -62;
    public static double SECOND_PICK_UP_Y = -64;
    public static double PICK_UP_HEADING = DROP_HEADING;
    public static double PICK_UP_TANGENT = Math.toRadians(270);

    // Chamber values
    public static double CHAMBER_TANGENT = Math.toRadians(90);
    public static double CHAMBER_X_SEPARATION = 3;
    public static double CHAMBER_X_CLOSE = 13;

    // Approacc first chamber pose
    public static double APPROACH_FIRST_CHAMBER_X = PRELOAD_CHAMBER_X - CHAMBER_X_SEPARATION;
    public static double APPROACH_FIRST_CHAMBER_Y = -39;
    public static double APPROACH_FIRST_CHAMBER_HEADING = PRELOAD_CHAMBER_HEADING;

    // First chamber pose
    public static double FIRST_CHAMBER_X = APPROACH_FIRST_CHAMBER_X;
    public static double FIRST_CHAMBER_Y = -31;
    public static double FIRST_CHAMBER_HEADING = APPROACH_FIRST_CHAMBER_HEADING;

    // Approach second chamber pose
    public static double APPROACH_SECOND_CHAMBER_X = FIRST_CHAMBER_X - CHAMBER_X_SEPARATION;
    public static double APPROACH_SECOND_CHAMBER_Y = APPROACH_FIRST_CHAMBER_Y;
    public static double APPROACH_SECOND_CHAMBER_HEADING = PRELOAD_CHAMBER_HEADING;

    // Second chamber pose
    public static double SECOND_CHAMBER_X = APPROACH_SECOND_CHAMBER_X;
    public static double SECOND_CHAMBER_Y = FIRST_CHAMBER_Y;
    public static double SECOND_CHAMBER_HEADING = APPROACH_SECOND_CHAMBER_HEADING;

    // Approach third chamber pose
    public static double APPROACH_THIRD_CHAMBER_X = SECOND_CHAMBER_X - CHAMBER_X_SEPARATION;
    public static double APPROACH_THIRD_CHAMBER_Y = APPROACH_FIRST_CHAMBER_Y;
    public static double APPROACH_THIRD_CHAMBER_HEADING = PRELOAD_CHAMBER_HEADING;

    // Third chamber pose
    public static double THIRD_CHAMBER_X = APPROACH_THIRD_CHAMBER_X;
    public static double THIRD_CHAMBER_Y = FIRST_CHAMBER_Y;
    public static double THIRD_CHAMBER_HEADING = APPROACH_THIRD_CHAMBER_HEADING;

    // Velocities
    public static double FAST_VELOCITY = 38;
    public static double MEDIUM_VELOCITY = 26;
    public static double SLOW_VELOCITY = 20;

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

        // Construct a preload velocity constraint.
        TranslationalVelConstraint preloadVelocityConstraint =
                new TranslationalVelConstraint(SLOW_VELOCITY);

        // Construct a first spike mark velocity constraint.
        VelConstraint firstSpikeMarkVelocityConstraint = (robotPose, _path, _disp) ->
                robotPose.position.x.value() > 35 ? MEDIUM_VELOCITY : FAST_VELOCITY;

        // Construct a pick up velocity constraint.
        VelConstraint pickUpVelocityConstraint = (robotPose, _path, _disp) ->
                robotPose.position.y.value() < -52 ? SLOW_VELOCITY : FAST_VELOCITY;

        // Construct a chamber velocity constraints.
        VelConstraint firstChamberVelocityConstraint = (robotPose, _path, _disp) ->
                robotPose.position.x.value() < FIRST_CHAMBER_X + CHAMBER_X_CLOSE ? MEDIUM_VELOCITY : FAST_VELOCITY;
        VelConstraint secondChamberVelocityConstraint = (robotPose, _path, _disp) ->
                robotPose.position.x.value() < SECOND_CHAMBER_X + CHAMBER_X_CLOSE ? MEDIUM_VELOCITY : FAST_VELOCITY;
        VelConstraint thirdChamberVelocityConstraint = (robotPose, _path, _disp) ->
                robotPose.position.x.value() < THIRD_CHAMBER_X + CHAMBER_X_CLOSE ? MEDIUM_VELOCITY : FAST_VELOCITY;

        // Construct poses.
        //////////////////////////////////////////////////////////////////////

        // Construct a chamber pose.
        Pose2d chamberPose = new Pose2d(PRELOAD_CHAMBER_X, PRELOAD_CHAMBER_Y, PRELOAD_CHAMBER_HEADING);

        // Construct a first spike mark pose.
        Pose2d firstSpikeMarkPose = new Pose2d(FIRST_SPIKE_MARK_X, FIRST_SPIKE_MARK_Y, FIRST_SPIKE_MARK_HEADING);

        // Construct a  drop pose.
        Pose2d dropPose = new Pose2d(DROP_X, DROP_Y, DROP_HEADING);

        // Construct a second spike mark pose.
        Pose2d secondSpikeMarkPose = new Pose2d(SECOND_SPIKE_MARK_X, SECOND_SPIKE_MARK_Y, SECOND_SPIKE_MARK_HEADING);

        // Construct an approach pick up pose.
        Pose2d approachPickUpPose = new Pose2d(APPROACH_PICK_UP_X, APPROACH_PICK_UP_Y, APPROACH_PICK_UP_HEADING);

        // Construct a first pick up pose.
        Pose2d firstPickUpPose = new Pose2d(PICK_UP_X, FIRST_PICK_UP_Y, PICK_UP_HEADING);

        // Construct a second pick up pose.
        Pose2d secondPickUpPose = new Pose2d(PICK_UP_X, SECOND_PICK_UP_Y, PICK_UP_HEADING);

        // Construct an approach first chamber pose.
        Pose2d approachFirstChamberPose = new Pose2d(APPROACH_FIRST_CHAMBER_X, APPROACH_FIRST_CHAMBER_Y, APPROACH_FIRST_CHAMBER_HEADING);

        // Construct a first chamber pose.
        Pose2d firstChamberPose = new Pose2d(FIRST_CHAMBER_X, FIRST_CHAMBER_Y, FIRST_CHAMBER_HEADING);

        // Construct an approach second chamber pose.
        Pose2d approachSecondChamberPose = new Pose2d(APPROACH_SECOND_CHAMBER_X, APPROACH_SECOND_CHAMBER_Y, APPROACH_SECOND_CHAMBER_HEADING);

        // Construct a second chamber pose.
        Pose2d secondChamberPose = new Pose2d(SECOND_CHAMBER_X, SECOND_CHAMBER_Y, SECOND_CHAMBER_HEADING);

        // Construct an approach third chamber pose.
        Pose2d approachThirdChamberPose = new Pose2d(APPROACH_THIRD_CHAMBER_X, APPROACH_THIRD_CHAMBER_Y, APPROACH_THIRD_CHAMBER_HEADING);

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

        // Construct a drive from first spike mark to drop to second spike mark trajectory.
        TrajectoryActionBuilder driveFromFirstSpikeMarkToDropToSecondSpikeMarkBuilder = driveFromChamberToFirstSpikeMarkBuilder.endTrajectory().fresh()
                .strafeTo(dropPose.position)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(secondSpikeMarkPose.position, SECOND_SPIKE_MARK_END_TANGENT);
        Action driveFromFirstSpikeMarkToDropToSecondSpikeMark = driveFromFirstSpikeMarkToDropToSecondSpikeMarkBuilder.build();

        // Construct a drive from second spike mark to pick up trajectory.
        TrajectoryActionBuilder driveFromSecondSpikeMarkToPickUpBuilder = driveFromFirstSpikeMarkToDropToSecondSpikeMarkBuilder.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(approachPickUpPose.position, APPROACH_PICK_UP_TANGENT, pickUpVelocityConstraint)
                .splineToConstantHeading(firstPickUpPose.position, PICK_UP_TANGENT, pickUpVelocityConstraint);
        Action driveFromSecondSpikeMarkToPickUp = driveFromSecondSpikeMarkToPickUpBuilder.build();

        // Construct a drive from pick up to first chamber trajectory.
        TrajectoryActionBuilder driveFromPickUpToFirstChamberBuilder = driveFromSecondSpikeMarkToPickUpBuilder.endTrajectory().fresh()
                .setTangent(CHAMBER_TANGENT)
                .splineToConstantHeading(approachFirstChamberPose.position, CHAMBER_TANGENT, firstChamberVelocityConstraint)
                .splineToConstantHeading(firstChamberPose.position, CHAMBER_TANGENT, firstChamberVelocityConstraint);
        Action driveFromPickUpToFirstChamber = driveFromPickUpToFirstChamberBuilder.build();

        // Construct a drive from first chamber to pick up trajectory.
        TrajectoryActionBuilder driveFromFirstChamberToPickUpBuilder = driveFromPickUpToFirstChamberBuilder.endTrajectory().fresh()
                .setTangent(PICK_UP_TANGENT)
                .splineToConstantHeading(approachPickUpPose.position, APPROACH_PICK_UP_TANGENT, pickUpVelocityConstraint)
                .splineToConstantHeading(firstPickUpPose.position, PICK_UP_TANGENT, pickUpVelocityConstraint);
        Action driveFromFirstChamberToPickUp = driveFromFirstChamberToPickUpBuilder.build();

        // Construct a drive from pick up to second chamber trajectory.
        TrajectoryActionBuilder driveFromPickUpToSecondChamberBuilder = driveFromFirstChamberToPickUpBuilder.endTrajectory().fresh()
                .setTangent(CHAMBER_TANGENT)
                .splineToConstantHeading(approachSecondChamberPose.position, CHAMBER_TANGENT, secondChamberVelocityConstraint)
                .splineToConstantHeading(secondChamberPose.position, CHAMBER_TANGENT, secondChamberVelocityConstraint);
        Action driveFromPickUpToSecondChamber = driveFromPickUpToSecondChamberBuilder.build();

        // Construct a drive from second chamber to pick up trajectory.
        TrajectoryActionBuilder driveFromSecondChamberToPickUpBuilder = driveFromPickUpToSecondChamberBuilder.endTrajectory().fresh()
                .setTangent(PICK_UP_TANGENT)
                .splineToConstantHeading(approachPickUpPose.position, APPROACH_PICK_UP_TANGENT, pickUpVelocityConstraint)
                .splineToConstantHeading(secondPickUpPose.position, PICK_UP_TANGENT, pickUpVelocityConstraint);
        Action driveFromSecondChamberToPickUp = driveFromSecondChamberToPickUpBuilder.build();

        // Construct a drive from pick up to third chamber trajectory.
        TrajectoryActionBuilder driveFromPickUpToThirdChamberBuilder = driveFromSecondChamberToPickUpBuilder.endTrajectory().fresh()
                .setTangent(CHAMBER_TANGENT)
                .splineToConstantHeading(approachThirdChamberPose.position, CHAMBER_TANGENT, thirdChamberVelocityConstraint)
                .splineToConstantHeading(thirdChamberPose.position, CHAMBER_TANGENT, thirdChamberVelocityConstraint);
        Action driveFromPickUpToThirdChamber = driveFromPickUpToThirdChamberBuilder.build();

        // Construct a drive nowhere trajectory.
        Action driveNowhere = new SequentialAction();

        // Construct a main action.
        //////////////////////////////////////////////////////////////////////

        // Get the tall walls value.
        boolean tallWalls = robotHardware.getTallWalls();

        // Construct a main action.
        Action mainAction = new SequentialAction(

                // Score the preloaded specimen and drive to the first spike mark.
                scoreAndDrive(driveFromStartToChamber, driveFromChamberToFirstSpikeMark, Arm.AUTO_SPECIMEN_POSITION, true),

                // Grab the first spike mark sample.
                grabSample(),

                // Deliver the first spike mark sample and grab the second spike mark sample.
                new ParallelAction(
                        driveFromFirstSpikeMarkToDropToSecondSpikeMark,
                        new InstantAction(() -> robotHardware.setMinimumExtension()),
                        new MoveArm(robotHardware, Arm.WALL_POSITION, MoveArm.MEDIUM_INCREMENT),
                        new InstantAction(() -> robotHardware.setWristWallPosition(tallWalls)),
                        new SequentialAction(
                                new WaitForTime(1000),
                                new InstantAction(() -> robotHardware.openBigClaw()),
                                new WaitForTime(200),
                                new InstantAction(() -> robotHardware.setWristSubmersiblePosition()),
                                new InstantAction(() -> robotHardware.setAutoExtension()),
                                new MoveArm(robotHardware, Arm.AUTO_SPECIMEN_POSITION, MoveArm.MEDIUM_INCREMENT),
                                new WaitForTime(100)
                        )
                ),

                // Grab the first spike mark sample.
                grabSample(),

                // Deliver the second spike mark sample and drive to the pick up location.
                new ParallelAction(
                        driveFromSecondSpikeMarkToPickUp,
                        new InstantAction(() -> robotHardware.setMinimumExtension()),
                        new MoveArm(robotHardware, Arm.WALL_POSITION, MoveArm.MEDIUM_INCREMENT),
                        new InstantAction(() -> robotHardware.setWristWallPosition(tallWalls)),
                        new SequentialAction(
                                new WaitForTime(1000),
                                new InstantAction(() -> robotHardware.openBigClaw())
                        )
                ),

                // Pick up specimen.
                pickUpSpecimen(),

                // Score the first specimen and drive to the wall.
                scoreAndDrive(driveFromPickUpToFirstChamber, driveFromFirstChamberToPickUp, Arm.WALL_POSITION, false),

                // Pick up specimen.
                pickUpSpecimen(),

                // Score the second specimen and drive to the wall.
                scoreAndDrive(driveFromPickUpToSecondChamber, driveFromSecondChamberToPickUp, Arm.WALL_POSITION, false),

                // Pick up specimen.
                pickUpSpecimen(),

                // Score the third specimen.
                scoreAndDrive(driveFromPickUpToThirdChamber, driveNowhere, Arm.WALL_POSITION, false)

        );

        // Return the main action.
        return mainAction;

    }

    // Scores a specimen and then drives to a destination.
    private Action scoreAndDrive(Action driveToChamber, Action driveToDestination, int finalArmPosition, boolean isPreload) {

        // Get the tall walls value.
        boolean tallWalls = robotHardware.getTallWalls();

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
                new InstantAction(() -> robotHardware.setWristWallPosition(tallWalls)),

                // Drive to the destination while lowering the arm.
                new ParallelAction(
                        driveToDestination,
                        new SequentialAction(
                                new WaitForTime(isPreload ? 1000 : 800),
                                new ParallelAction(
                                        new InstantAction(() -> robotHardware.setLiftGroundPosition()),
                                        finalArmPosition == Arm.WALL_POSITION ?
                                                new InstantAction(() -> robotHardware.setMinimumExtension()) :
                                                new InstantAction(() -> robotHardware.setAutoExtension()),
                                        new InstantAction(() -> robotHardware.openBigClaw()),
                                        new InstantAction(() -> robotHardware.swivelSetHorizontal()),
                                        new MoveArm(robotHardware, finalArmPosition, MoveArm.MEDIUM_INCREMENT),
                                        finalArmPosition == Arm.WALL_POSITION ?
                                                new InstantAction(() -> robotHardware.setWristWallPosition(tallWalls)) :
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
                new MoveArm(robotHardware, Arm.SUBMERSIBLE_GRAB_POSITION, MoveArm.MEDIUM_INCREMENT),
                new WaitForTime(100),
                new InstantAction(() -> robotHardware.closeBigClaw()),
                new WaitForTime(250)
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