package edu.edina.OpModes.Autonomous;

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

    // Chamber pose
    public static double CHAMBER_X = START_X;
    public static double CHAMBER_Y = -29;
    public static double CHAMBER_HEADING = START_HEADING;

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

    // First drop pose
    public static double FIRST_DROP_X = FIRST_SPIKE_MARK_X;
    public static double FIRST_DROP_Y = -49.5;
    public static double FIRST_DROP_HEADING = FIRST_SPIKE_MARK_HEADING;

    // Second drop pose
    public static double SECOND_DROP_X = SECOND_SPIKE_MARK_X;
    public static double SECOND_DROP_Y = FIRST_DROP_Y;
    public static double SECOND_DROP_HEADING = FIRST_DROP_HEADING;

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

        // Initialize the robot.
        //////////////////////////////////////////////////////////////////////

        // Get hardware.
        robotHardware = new RobotHardware(this);

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

        // Construct poses.
        //////////////////////////////////////////////////////////////////////

        // Construct a start pose.
        Pose2d startPose = new Pose2d(START_X, START_Y, START_HEADING);

        // Construct a chamber pose.
        Pose2d chamberPose = new Pose2d(CHAMBER_X, CHAMBER_Y, CHAMBER_HEADING);

        // Construct a first spike mark pose.
        Pose2d firstSpikeMarkPose = new Pose2d(FIRST_SPIKE_MARK_X, FIRST_SPIKE_MARK_Y, FIRST_SPIKE_MARK_HEADING);

        // Construct a first drop pose.
        Pose2d firstDropPose = new Pose2d(FIRST_DROP_X, FIRST_DROP_Y, FIRST_DROP_HEADING);

        // Construct a second spike mark pose.
        Pose2d secondSpikeMarkPose = new Pose2d(SECOND_SPIKE_MARK_X, SECOND_SPIKE_MARK_Y, SECOND_SPIKE_MARK_HEADING);

        // Construct a second drop pose.
        Pose2d secondDropPose = new Pose2d(SECOND_DROP_X, SECOND_DROP_Y, SECOND_DROP_HEADING);

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

        TrajectoryActionBuilder driveFromSecondSpikeMarkToSecondDropBuilder = drive.actionBuilder(secondSpikeMarkPose)
                .strafeTo(secondDropPose.position);
        Action driveFromSecondSpikeMarkToSecondDrop = driveFromSecondSpikeMarkToSecondDropBuilder.build();

        // Construct a main action.
        //////////////////////////////////////////////////////////////////////

        // Construct a main action.
        Action mainAction = new SequentialAction(

                // Score the preloaded specimen and drive to the first spike mark.
                scoreAndDrive(driveFromStartToChamber, driveFromChamberToFirstSpikeMark, false),

                // Grab the first spike mark sample.
                grabSample(),

                // Deliver the first spike mark sample to the human player.
                deliverSample(driveFromFirstSpikeMarkToFirstDrop),

                // Drive to the second spike mark.
                driveToSpikeMark(driveFromFirstDropToSecondSpikeMark, true),

                // Grab the second spike mark sample.
                grabSample(),

                // Deliver the second spike mark sample to the human player.
                deliverSample(driveFromSecondSpikeMarkToSecondDrop)

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
    private Action scoreAndDrive(Action driveToChamber, Action driveToDestination, boolean goToWall) {

        // Get the tall walls value.
        boolean tallWalls = robotHardware.getTallWalls();

        // Get an arm position
        int armPosition;
        if(goToWall) {
            armPosition = tallWalls ? Arm.SUBMERSIBLE_TO_TALL_WALL_POSITION : Arm.SUBMERSIBLE_TO_SHORT_WALL_POSITION;
        }
        else {
            armPosition = Arm.SUBMERSIBLE_ENTER_POSITION;
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
                                        goToWall ?
                                                new InstantAction(() -> robotHardware.setMinimumExtension()) :
                                                new InstantAction(() -> robotHardware.setAutoExtension()),
                                        new InstantAction(() -> robotHardware.openBigClaw()),
                                        new MoveArm(robotHardware, armPosition, true),
                                        goToWall ?
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
}