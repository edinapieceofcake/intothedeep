package edu.edina.OpModes.Autonomous;

import static edu.edina.Libraries.Robot.RobotHardware.BLUE_SQUARE;
import static edu.edina.Libraries.Robot.RobotHardware.getBanner;
import static edu.edina.Libraries.Robot.RobotHardware.getSymbol;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
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
    public static double SECOND_SPIKE_MARK_X = 55.5;
    public static double SECOND_SPIKE_MARK_Y = FIRST_SPIKE_MARK_Y;
    public static double SECOND_SPIKE_MARK_HEADING = FIRST_SPIKE_MARK_HEADING;
    public static double SECOND_SPIKE_MARK_END_TANGENT = FIRST_SPIKE_MARK_END_TANGENT;

    // Third spike mark pose
    public static double THIRD_SPIKE_MARK_X = 53;
    public static double THIRD_SPIKE_MARK_Y = -25.5;
    public static double THIRD_SPIKE_MARK_HEADING = Math.toRadians(180);
    public static double THIRD_SPIKE_MARK_END_TANGENT = Math.toRadians(0);

    // First drop pose
    public static double FIRST_DROP_X = FIRST_SPIKE_MARK_X;
    public static double FIRST_DROP_Y = -49.5;
    public static double FIRST_DROP_HEADING = FIRST_SPIKE_MARK_HEADING;

    // Second drop pose
    public static double SECOND_DROP_X = SECOND_SPIKE_MARK_X;
    public static double SECOND_DROP_Y = FIRST_DROP_Y;
    public static double SECOND_DROP_HEADING = FIRST_DROP_HEADING;

    // Third drop pose
    public static double THIRD_DROP_X = 40;
    public static double THIRD_DROP_Y = FIRST_DROP_Y;
    public static double THIRD_DROP_HEADING = FIRST_DROP_HEADING;

    // Pick up pose
    public static double PICK_UP_X = THIRD_DROP_X;
    public static double PICK_UP_Y = -55;
    public static double PICK_UP_HEADING = FIRST_DROP_HEADING;
    public static double PICK_UP_START_TANGENT = Math.toRadians(90);
    public static double PICK_UP_END_TANGENT = Math.toRadians(270);

    // Chamber tangents
    public static double CHAMBER_START_TANGENT = Math.toRadians(270);
    public static double CHAMBER_END_TANGENT = Math.toRadians(90);

    // First chamber pose
    public static double FIRST_CHAMBER_X = 11;
    public static double FIRST_CHAMBER_Y = PRELOAD_CHAMBER_Y;
    public static double FIRST_CHAMBER_HEADING = PRELOAD_CHAMBER_HEADING;

    // Second chamber pose
    public static double SECOND_CHAMBER_X = 9;
    public static double SECOND_CHAMBER_Y = PRELOAD_CHAMBER_Y;
    public static double SECOND_CHAMBER_HEADING = PRELOAD_CHAMBER_HEADING;

    // Third chamber pose
    public static double THIRD_CHAMBER_X = 7;
    public static double THIRD_CHAMBER_Y = PRELOAD_CHAMBER_Y;
    public static double THIRD_CHAMBER_HEADING = PRELOAD_CHAMBER_HEADING;

    // Fourth chamber pose
    public static double FOURTH_CHAMBER_X = 5;
    public static double FOURTH_CHAMBER_Y = PRELOAD_CHAMBER_Y;
    public static double FOURTH_CHAMBER_HEADING = PRELOAD_CHAMBER_HEADING;

    // Velocities
    public static double FAST_VELOCITY = 35;
    public static double MEDIUM_VELOCITY = 23;
    public static double SLOW_VELOCITY = 15;

    // Score y coordinate
    public static double SCORE_Y = -42;

    // Wall x coordinate
    public static double WALL_X = 39;

    // Wall y coordinate
    public static double WALL_Y = -56;

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
                prompt("Walls", "X = Tall, B = Short");

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

        // Get the main action.
        Action mainAction = getMainAction();

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

    // Determines whether the robot is close to the first spike mark.
    private static boolean isCloseToFirstSpikeMark(Pose2dDual robotPose) {
        return robotPose.position.x.value() > 35;
    }

    // Gets the main action.
    private Action getMainAction() {

        // Construct velocity constraints.
        //////////////////////////////////////////////////////////////////////

        // Construct a first spike mark velocity constraint.
        VelConstraint firstSpikeMarkVelocityConstraint = (robotPose, _path, _disp) -> {

            // Determine whether the robot is close to the first spike mark.
            boolean closeToFirstSpikeMark = isCloseToFirstSpikeMark(robotPose);

            // If the robot is close to the first spike mark...
            if (closeToFirstSpikeMark) {

                // Go slow.
                return MEDIUM_VELOCITY;

            }

            // Otherwise (if the robot is far from the first spike mark)...
            else {

                // Go fast.
                return FAST_VELOCITY;

            }

        };

        // Construct a preload velocity constraint.
        TranslationalVelConstraint preloadVelocityConstraint = new TranslationalVelConstraint(MEDIUM_VELOCITY);

        // Construct a chamber velocity constraint.
        TranslationalVelConstraint chamberVelocityConstraint = new TranslationalVelConstraint(FAST_VELOCITY);

        // Construct poses.
        //////////////////////////////////////////////////////////////////////

        // Construct a start pose.
        Pose2d startPose = new Pose2d(START_X, START_Y, START_HEADING);

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

        // Construct a third spike mark pose.
        Pose2d thirdSpikeMarkPose = new Pose2d(THIRD_SPIKE_MARK_X, THIRD_SPIKE_MARK_Y, THIRD_SPIKE_MARK_HEADING);

        // Construct a third drop pose.
        Pose2d thirdDropPose = new Pose2d(THIRD_DROP_X, THIRD_DROP_Y, THIRD_DROP_HEADING);

        // Construct a pick up pose.
        Pose2d pickUpPose = new Pose2d(PICK_UP_X, PICK_UP_Y, PICK_UP_HEADING);

        // Construct a first chamber pose.
        Pose2d firstChamberPose = new Pose2d(FIRST_CHAMBER_X, FIRST_CHAMBER_Y, FIRST_CHAMBER_HEADING);

        // Construct a second chamber pose.
        Pose2d secondChamberPose = new Pose2d(SECOND_CHAMBER_X, SECOND_CHAMBER_Y, SECOND_CHAMBER_HEADING);

        // Construct a third chamber pose.
        Pose2d thirdChamberPose = new Pose2d(THIRD_CHAMBER_X, THIRD_CHAMBER_Y, THIRD_CHAMBER_HEADING);

        // Construct a fourth chamber pose.
        Pose2d fourthChamberPose = new Pose2d(FOURTH_CHAMBER_X, FOURTH_CHAMBER_Y, FOURTH_CHAMBER_HEADING);

        // Construct trajectories.
        //////////////////////////////////////////////////////////////////////

        // Construct a drive interface.
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Construct a drive from start to chamber trajectory.
        TrajectoryActionBuilder driveFromStartToChamberBuilder = drive.actionBuilder(startPose)
                .strafeTo(chamberPose.position, preloadVelocityConstraint);
        Action driveFromStartToChamber = driveFromStartToChamberBuilder.build();

        // Construct a drive from chamber to first spike mark trajectory.
        TrajectoryActionBuilder driveFromChamberToFirstSpikeMarkBuilder = drive.actionBuilder(chamberPose)
                .splineToConstantHeading(firstSpikeMarkPose.position, FIRST_SPIKE_MARK_END_TANGENT, firstSpikeMarkVelocityConstraint);
        Action driveFromChamberToFirstSpikeMark = driveFromChamberToFirstSpikeMarkBuilder.build();

        // Construct a drive from first spike mark to first drop trajectory.
        TrajectoryActionBuilder driveFromFirstSpikeMarkToFirstDropBuilder = drive.actionBuilder(firstSpikeMarkPose)
                .strafeTo(firstDropPose.position);
        Action driveFromFirstSpikeMarkToFirstDrop = driveFromFirstSpikeMarkToFirstDropBuilder.build();

        // Construct a drive from first drop to second spike mark trajectory.
        TrajectoryActionBuilder driveFromFirstDropToSecondSpikeMarkBuilder = drive.actionBuilder(firstDropPose)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(secondSpikeMarkPose.position, SECOND_SPIKE_MARK_END_TANGENT);
        Action driveFromFirstDropToSecondSpikeMark = driveFromFirstDropToSecondSpikeMarkBuilder.build();

        // Construct a drive from second spike mark to second drop trajectory.
        TrajectoryActionBuilder driveFromSecondSpikeMarkToSecondDropBuilder = drive.actionBuilder(secondSpikeMarkPose)
                .strafeTo(secondDropPose.position);
        Action driveFromSecondSpikeMarkToSecondDrop = driveFromSecondSpikeMarkToSecondDropBuilder.build();

        // Construct a drive from second drop to third spike mark trajectory.
        TrajectoryActionBuilder driveFromSecondDropToThirdSpikeMarkBuilder = drive.actionBuilder(secondDropPose)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(thirdSpikeMarkPose.position, THIRD_SPIKE_MARK_END_TANGENT);
        Action driveFromSecondDropToThirdSpikeMark = driveFromSecondDropToThirdSpikeMarkBuilder.build();

        // Construct a drive from third spike mark to third drop trajectory.
        TrajectoryActionBuilder driveFromThirdSpikeMarkToThirdDropBuilder = drive.actionBuilder(thirdSpikeMarkPose)
                .strafeTo(thirdDropPose.position);
        Action driveFromThirdSpikeMarkToThirdDrop = driveFromThirdSpikeMarkToThirdDropBuilder.build();

        // Construct a drive from third drop to pick up trajectory.
        TrajectoryActionBuilder driveFromThirdDropToPickUpBuilder = drive.actionBuilder(thirdDropPose)
                .strafeTo(pickUpPose.position);
        Action driveFromThirdDropToPickUp = driveFromThirdDropToPickUpBuilder.build();

        // Construct a drive from pick up to first chamber trajectory.
        TrajectoryActionBuilder driveFromPickUpToFirstChamberBuilder = drive.actionBuilder(pickUpPose)
                .setTangent(CHAMBER_START_TANGENT)
				.splineToConstantHeading(firstChamberPose.position, CHAMBER_END_TANGENT, chamberVelocityConstraint);
        Action driveFromPickUpToFirstChamber = driveFromPickUpToFirstChamberBuilder.build();

        // Construct a drive from first chamber to pick up trajectory.
        TrajectoryActionBuilder driveFromFirstChamberToPickUpBuilder = drive.actionBuilder(firstChamberPose)
                .setTangent(PICK_UP_START_TANGENT)
				.splineToConstantHeading(pickUpPose.position, PICK_UP_END_TANGENT, chamberVelocityConstraint);
        Action driveFromFirstChamberToPickUp = driveFromFirstChamberToPickUpBuilder.build();

        // Construct a drive from pick up to second chamber trajectory.
        TrajectoryActionBuilder driveFromPickUpToSecondChamberBuilder = drive.actionBuilder(pickUpPose)
                .setTangent(CHAMBER_START_TANGENT)
                .splineToConstantHeading(secondChamberPose.position, CHAMBER_END_TANGENT, chamberVelocityConstraint);
        Action driveFromPickUpToSecondChamber = driveFromPickUpToSecondChamberBuilder.build();

        // Construct a drive from second chamber to pick up trajectory.
        TrajectoryActionBuilder driveFromSecondChamberToPickUpBuilder = drive.actionBuilder(secondChamberPose)
                .setTangent(PICK_UP_START_TANGENT)
                .splineToConstantHeading(pickUpPose.position, PICK_UP_END_TANGENT, chamberVelocityConstraint);
        Action driveFromSecondChamberToPickUp = driveFromSecondChamberToPickUpBuilder.build();

        // Construct a drive from pick up to third chamber trajectory.
        TrajectoryActionBuilder driveFromPickUpToThirdChamberBuilder = drive.actionBuilder(pickUpPose)
                .setTangent(CHAMBER_START_TANGENT)
                .splineToConstantHeading(thirdChamberPose.position, CHAMBER_END_TANGENT, chamberVelocityConstraint);
        Action driveFromPickUpToThirdChamber = driveFromPickUpToThirdChamberBuilder.build();

        // Construct a drive from third chamber to pick up trajectory.
        TrajectoryActionBuilder driveFromThirdChamberToPickUpBuilder = drive.actionBuilder(thirdChamberPose)
                .setTangent(PICK_UP_START_TANGENT)
                .splineToConstantHeading(pickUpPose.position, PICK_UP_END_TANGENT, chamberVelocityConstraint);
        Action driveFromThirdChamberToPickUp = driveFromThirdChamberToPickUpBuilder.build();

        // Construct a drive from pick up to fourth chamber trajectory.
        TrajectoryActionBuilder driveFromPickUpToFourthChamberBuilder = drive.actionBuilder(pickUpPose)
                .setTangent(CHAMBER_START_TANGENT)
                .splineToConstantHeading(fourthChamberPose.position, CHAMBER_END_TANGENT, chamberVelocityConstraint);
        Action driveFromPickUpToFourthChamber = driveFromPickUpToFourthChamberBuilder.build();

        // Construct a drive no where trajectory.
        TrajectoryActionBuilder driveNoWhereBuilder = drive.actionBuilder(fourthChamberPose);
        Action driveNowhere = driveNoWhereBuilder.build();

        // Construct a main action.
        //////////////////////////////////////////////////////////////////////

        // Construct a main action.
        Action mainAction = new SequentialAction(

                // Score the preloaded specimen and drive to the first spike mark.
                scoreAndDrive(driveFromStartToChamber, driveFromChamberToFirstSpikeMark, false, false),

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

                // Drive to the third spike mark.
                driveToSpikeMark(driveFromSecondDropToThirdSpikeMark, false),

                // Grab the third spike mark sample.
                grabSample(),

                // Deliver the third spike mark sample to the human player.
                deliverSample(driveFromThirdSpikeMarkToThirdDrop),

                // Drive to pick up
                driveFromThirdDropToPickUp,
                new WaitForTime(250),

                // Pick up specimen
                pickUpSpecimen(),

                // Score the first specimen and drive to the wall.
                scoreAndDrive(driveFromPickUpToFirstChamber, driveFromFirstChamberToPickUp, true, false),

                // Pick up specimen
                pickUpSpecimen(),

                // Score the second specimen and drive to the wall.
                scoreAndDrive(driveFromPickUpToSecondChamber, driveFromSecondChamberToPickUp, true, false),

                // Pick up specimen
                pickUpSpecimen(),

                // Score the third specimen and drive to the wall.
                scoreAndDrive(driveFromPickUpToThirdChamber, driveFromThirdChamberToPickUp, true, false),

                // Pick up specimen
                pickUpSpecimen(),

                // Score the fourth specimen.
                scoreAndDrive(driveFromPickUpToFourthChamber, driveNowhere, false, true)

        );

        // Return the main action.
        return mainAction;

//		// Construct velocity constraints.
//		//////////////////////////////////////////////////////////////////////
//
//		// Construct a plow velocity constraint.
//		TranslationalVelConstraint plowVelocityConstraint = new TranslationalVelConstraint(FAST_VELOCITY);
//
//		// Construct a wall velocity constraint.
//		VelConstraint wallVelocityConstraint = (robotPose, _path, _disp) -> {
//
//			// Determine whether the robot is close to the chamber.
//			boolean closeToChamber = isCloseToChamber(robotPose);
//
//			// Determine whether the robot is close to the wall.
//			boolean closeToWall = isCloseToWall(robotPose);
//
//			// If the robot is close to the chamber or wall...
//			if (closeToChamber || closeToWall) {
//
//				// Go slow.
//				return SLOW_VELOCITY;
//
//			}
//
//			// Otherwise (if the robot is far from the chamber and wall)...
//			else {
//
//				// Go fast.
//				return FAST_VELOCITY;
//
//			}
//
//		};
//
//		// Construct a chamber velocity constraint.
//		VelConstraint chamberVelocityConstraint = (robotPose, _path, _disp) -> {
//
//			// Determine whether the robot is close to the chamber.
//			boolean closeToChamber = isCloseToChamber(robotPose);
//
//			// If the robot is close to the chamber...
//			if (closeToChamber) {
//
//				// Go slow.
//				return SLOW_VELOCITY;
//
//			}
//
//			// Otherwise (if the robot is far from the chamber)...
//			else {
//
//				// Go fast.
//				return FAST_VELOCITY;
//
//			}
//
//		};
//
//		// Construct trajectories.
//		//////////////////////////////////////////////////////////////////////
//
//		// Construct a start pose.
//		Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(270));
//
//		// Construct a drive interface.
//		MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
//
//		// Construct a first drive to chamber action.
//		TrajectoryActionBuilder driveToChamber1Builder = drive.actionBuilder(startPose)
//				.strafeTo(new Vector2d(START_X, CHAMBER_Y));
//		Action driveToChamber1 = driveToChamber1Builder.build();
//
//		// Construct first plow action.
//		TrajectoryActionBuilder plow1Builder = driveToChamber1Builder.endTrajectory().fresh()
//				.setTangent(Math.toRadians(270))
//				.splineToConstantHeading(new Vector2d(36, -24), Math.toRadians(90), plowVelocityConstraint)
//				.splineToConstantHeading(new Vector2d(42, -14), Math.toRadians(0), plowVelocityConstraint)
//				.splineToConstantHeading(new Vector2d(50, -51), Math.toRadians(270), plowVelocityConstraint);
//		Action plow1 = plow1Builder.build();
//
//		// Construct second plow action.
//		TrajectoryActionBuilder plow2Builder = plow1Builder.endTrajectory().fresh()
//				.setTangent(Math.toRadians(90))
//				.splineToConstantHeading(new Vector2d(52, -14), Math.toRadians(0), plowVelocityConstraint)
//				.splineToConstantHeading(new Vector2d(WALL_X, -49), Math.toRadians(270), plowVelocityConstraint)
//				.lineToY(WALL_Y, wallVelocityConstraint);
//		Action plow2 = plow2Builder.build();
//
//		// Construct third plow action.
//		TrajectoryActionBuilder plow3Builder = plow2Builder.endTrajectory().fresh()
//				.setTangent(Math.toRadians(270))
//				.splineToConstantHeading(new Vector2d(59,-13), Math.toRadians(0))
//				.setTangent(Math.toRadians(0))
//				.splineToConstantHeading(new Vector2d(61,-15), Math.toRadians(270))
//				.splineToConstantHeading(new Vector2d(61,-49), Math.toRadians(270))
//				.lineToY(WALL_Y, wallVelocityConstraint);
//		Action plow3 = plow3Builder.build();
//
//		// Construct a second drive to chamber action.
//		TrajectoryActionBuilder driveToChamber2Builder = plow3Builder.endTrajectory().fresh()
//				.setTangent(Math.toRadians(90))
//				.splineToConstantHeading(new Vector2d(3, CHAMBER_Y), Math.toRadians(90), chamberVelocityConstraint);
//		Action driveToChamber2 = driveToChamber2Builder.build();
//
//		// Construct a first drive to first wall action.
//		TrajectoryActionBuilder driveToWall1Builder = driveToChamber2Builder.endTrajectory().fresh()
//				.setTangent(Math.toRadians(270))
//				.splineToConstantHeading(new Vector2d(WALL_X, WALL_Y), Math.toRadians(270), wallVelocityConstraint);
//		Action driveToWall1 = driveToWall1Builder.build();
//
//		// Construct a third drive to chamber action.
//		TrajectoryActionBuilder driveToChamber3Builder = driveToWall1Builder.endTrajectory().fresh()
//				.setTangent(Math.toRadians(90))
//				.splineToConstantHeading(new Vector2d(0, CHAMBER_Y), Math.toRadians(90), chamberVelocityConstraint);
//		Action driveToChamber3 = driveToChamber3Builder.build();
//
//		// Construct a second drive to wall action.
//		TrajectoryActionBuilder driveToWall2Builder = driveToChamber3Builder.endTrajectory().fresh()
//				.setTangent(Math.toRadians(270))
//				.splineToConstantHeading(new Vector2d(WALL_X, WALL_Y), Math.toRadians(270), wallVelocityConstraint);
//		Action driveToWall2 = driveToWall2Builder.build();
//
//		// Construct a fourth drive to chamber action.
//		TrajectoryActionBuilder driveToChamber4Builder = driveToWall2Builder.endTrajectory().fresh()
//				.setTangent(Math.toRadians(90))
//				.splineToConstantHeading(new Vector2d(-3, CHAMBER_Y), Math.toRadians(90), chamberVelocityConstraint);
//		Action driveToChamber4 = driveToChamber4Builder.build();
//
//		// Construct a fourth drive to score action.
//		TrajectoryActionBuilder driveToScore4Builder = driveToChamber4Builder.endTrajectory().fresh()
//				.strafeTo(new Vector2d(-3, SCORE_Y));
//		Action driveToScore4 = driveToScore4Builder.build();
//
//		// Construct a third drive to wall action.
//		TrajectoryActionBuilder driveToWall3Builder = driveToChamber4Builder.endTrajectory().fresh()
//				.setTangent(Math.toRadians(270))
//				.splineToConstantHeading(new Vector2d(WALL_X, WALL_Y), Math.toRadians(270), wallVelocityConstraint);
//		Action driveToWall3 = driveToWall3Builder.build();
//
//		// Construct a fifth drive to chamber action.
//		TrajectoryActionBuilder driveToChamber5Builder = driveToWall3Builder.endTrajectory().fresh()
//				.setTangent(Math.toRadians(90))
//				.splineToConstantHeading(new Vector2d(-6, CHAMBER_Y), Math.toRadians(90), chamberVelocityConstraint);
//		Action driveToChamber5 = driveToChamber5Builder.build();
//
//		// Construct a fifth drive to score action.
//		TrajectoryActionBuilder driveToScore5Builder = driveToChamber5Builder.endTrajectory().fresh()
//				.strafeTo(new Vector2d(-6, SCORE_Y));
//		Action driveToScore5 = driveToScore5Builder.build();
//
//		// Construct a drive to park action.
//		TrajectoryActionBuilder driveToParkBuilder = driveToScore5Builder.endTrajectory().fresh()
//				.strafeTo(new Vector2d(54, -54));
//		Action driveToPark = driveToParkBuilder.build();
//
//		// Construct a main action.
//		//////////////////////////////////////////////////////////////////////
//
//		// Construct a main action.
        // Action mainAction = new SequentialAction(
//
//				// Drive to the chamber while raising the arm.
//				raiseArmToChamberAndDrive(driveToChamber1, false, true),
//
//				// Score the preloaded specimen and plow the first sample.
//				new ParallelAction(
//						scoreSpecimen(plow1, robotHardware),
//						new SequentialAction(
//								new WaitForTime(1000),
//								lowerArmFromChamber(true)
//						)
//				),
//
//				// Plow the second sample.
//				plow2,
//
//				// Plow the third sample.
//				plow3,
//
//				// Grab and score the first wall specimen.
//				grabAndScoreSpecimen(driveToChamber2, driveToWall1),
//
//				// Grab and score the second wall specimen.
//				grabAndScoreSpecimen(driveToChamber3, driveToWall2),
//
//				// Grab and score the third wall specimen.
//				grabAndScoreSpecimen(driveToChamber4, driveToWall3),
//
//				// Grab the fourth wall specimen.
//				grabSpecimen(),
//
//				// Drive to the chamber while raising the arm.
//				raiseArmToChamberAndDrive(driveToChamber5, true, false),
//
//				// Score the third wall specimen.
//				scoreSpecimen(driveToScore5, robotHardware),
//
//				// Lower the arm.
//				lowerArmFromChamber(false)
//
//				// Drive to park while lowering the arm.
////				new ParallelAction(
////						driveToPark,
////						lowerArmFromChamber(false)
////				)

//		);
//
//        // Return the main action.
//        return mainAction;

    }

    // Scores a specimen and then drives to a destination.
    private Action scoreAndDrive(Action driveToChamber, Action driveToDestination, boolean goToWall, boolean lower) {

        // Get the tall walls value.
        boolean tallWalls = robotHardware.getTallWalls();

        // Get an arm position
        int armPosition;
        if(goToWall) {
            armPosition = tallWalls ? Arm.SUBMERSIBLE_TO_TALL_WALL_POSITION : Arm.SUBMERSIBLE_TO_SHORT_WALL_POSITION;
        }
        else {
            armPosition = lower ? Arm.MINIMUM_POSITION : Arm.SUBMERSIBLE_ENTER_POSITION;
        }

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
                new WaitForTime(200),
                new InstantAction(() -> robotHardware.setWristWallPosition()),

                // Drive to the destination while lowering the arm.
                new ParallelAction(
                        driveToDestination,
                        new SequentialAction(
                                new WaitForTime(1000),
                                new ParallelAction(
                                        new InstantAction(() -> robotHardware.setLiftGroundPosition()),
                                        goToWall || lower ?
                                                new InstantAction(() -> robotHardware.setMinimumExtension()) :
                                                new InstantAction(() -> robotHardware.setAutoExtension()),
                                        new InstantAction(() -> robotHardware.openBigClaw()),
                                        new MoveArm(robotHardware, armPosition, true),
                                        goToWall && !lower ?
                                                new InstantAction(() -> robotHardware.setWristWallPosition()) :
                                                new InstantAction(() -> robotHardware.setWristSubmersiblePosition())
                                )
                        )
                ),

                // Wait for the arm to settle.
                new WaitForTime(200)

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

        // Get the tall walls value.
        boolean tallWalls = robotHardware.getTallWalls();

        // Get an arm position.
        int armPosition = tallWalls ? Arm.SUBMERSIBLE_TO_TALL_WALL_POSITION : Arm.SUBMERSIBLE_TO_SHORT_WALL_POSITION;

        // Construct an action.
        Action action = new SequentialAction(
                new ParallelAction(
                        new InstantAction(() -> robotHardware.setMinimumExtension()),
                        new MoveArm(robotHardware, armPosition, true),
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
                                new InstantAction(() -> robotHardware.swivelSetHorizontal()) :
                                new InstantAction(() -> robotHardware.swivelSetVertical()),
                        new InstantAction(() -> robotHardware.setAutoExtension()),
                        new InstantAction(() -> robotHardware.setWristSubmersiblePosition()),
                        new MoveArm(robotHardware, Arm.SUBMERSIBLE_ENTER_POSITION, true),
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

	/*
    // Raises arm the arm to the chamber and drives.
    private Action raiseArmToChamberAndDrive(Action drive, boolean delayDriving, boolean moveArmFast) {

        // Get a drive delay.
        int driveDelay = delayDriving ? 250 : 0;

        // Construct an action.
        Action action = new ParallelAction(
                new ParallelAction(
                        new InstantAction(() -> robotHardware.setWristChamberPosition()),
                        new SequentialAction(
                                new WaitForTime(500),
                                new InstantAction(() -> robotHardware.swivelSetClipNoDelay())
                        ),
                        new MoveArm(robotHardware, Arm.CHAMBER_POSITION, moveArmFast)
                ),
                new SequentialAction(
                        new WaitForTime(driveDelay),
                        drive
                )
        );

        // Return the action.
        return action;

    }

    // Lowers arm from chamber.
    private Action lowerArmFromChamber(boolean endAtWall) {

        // Get the tall walls value.
        boolean tallWalls = robotHardware.getTallWalls();

        // Get an arm position.
        int armPosition;
        if (endAtWall) {
            armPosition = tallWalls ? Arm.SUBMERSIBLE_TO_TALL_WALL_POSITION : Arm.SUBMERSIBLE_TO_SHORT_WALL_POSITION;
        } else {
            armPosition = Arm.GROUND_POSITION;
        }

        // Construct an action.
        Action action = new SequentialAction(
                new WaitForTime(500),
                new InstantAction(() -> robotHardware.swivelSetHorizontal()),
                new InstantAction(endAtWall ? () -> robotHardware.setWristWallPosition() : () -> robotHardware.setWristSubmersiblePosition()),
                new MoveArm(robotHardware, armPosition, false),
                new WaitForHardware(robotHardware, TIMEOUT_MILLISECONDS)
        );

        // Return the action.
        return action;

    }

    // Scores a specimen.
    public static Action scoreSpecimen(Action backup, RobotHardware robotHardware) {

        // Construct a score specimen action.
        Action action = new ParallelAction(
                new InstantAction(() -> robotHardware.setWristChamberPosition()),
                new SequentialAction(
                        //new WaitForTime(200),
                        backup
                ),
                new SequentialAction(
                        new WaitForTime(400),
                        new InstantAction(() -> robotHardware.openClaws())
                )
        );

        // Return the action.
        return action;

    }

    // Determines whether the robot is close to the chamber.
    private static boolean isCloseToChamber(Pose2dDual robotPose) {
        return robotPose.position.y.value() > -40;
    }

    // Determines whether the robot is close to the wall.
    private static boolean isCloseToWall(Pose2dDual robotPose) {
        return robotPose.position.y.value() < -46;
    }

    // Grabs a specimen.
    private Action grabSpecimen() {

        // Construct an action.
        Action action = new SequentialAction(
                new InstantAction(() -> robotHardware.closeSmallClaw()),
                new WaitForTime(400)
        );

        // Return the action.
        return action;

    }

    // Grabs and scores a specimen.
    private Action grabAndScoreSpecimen(Action driveToChamber, Action driveToWall) {

        // Construct an action.
        Action action = new SequentialAction(

                // Grab the specimen.
                grabSpecimen(),

                // Drive to the chamber while raising the arm.
                raiseArmToChamberAndDrive(driveToChamber, true, false),

                // Score the wall specimen.
                new ParallelAction(

                        // Score the specimen and then drive to the wall.
                        scoreSpecimen(driveToWall, robotHardware),

                        // Lower the arm from the chamber.
                        new SequentialAction(
                                new WaitForTime(200),
                                lowerArmFromChamber(true)
                        ),

                        // Set the wrist to the wall position.
                        new SequentialAction(
                                new WaitForTime(800),
                                new InstantAction(() -> robotHardware.setWristWallPosition())
                        )

                )

        );

        // Return the action.
        return action;

    }
	*/
    // Adds telemetry.
    private void addTelemetry() {

        // Get a banner.
        String banner = getBanner(BLUE_SQUARE);

        boolean tallWalls = robotHardware.getTallWalls();

        String tallWallsSymbol = getSymbol(tallWalls);

        // Display main telemetry.
        telemetry.addData("AutoSpecimen", banner);
        telemetry.addData("- Tall Walls", tallWallsSymbol);

    }
    private void prompt(String caption, String value) {
        telemetry.addData(caption, value);
        telemetry.update();
    }

}