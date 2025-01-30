package edu.edina.OpModes.Autonomous;

import static edu.edina.OpModes.Autonomous.AutoSample.TIMEOUT_MILLISECONDS;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
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
public class AutoSpecimen extends LinearOpMode {

    // Chamber y coordinate
    public static double CHAMBER_Y = -29;

    // Fast velocity
    public static double FAST_VELOCITY = 27;

    // Score y coordinate
    public static double SCORE_Y = -42;

    // Slow velocity
    public static double SLOW_VELOCITY = 15;

    // Start x coordinate
    public static double START_X = 3;

    // Start y coordinate
    public static double START_Y = -60;

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

        // Open the big claw.
        robotHardware.openBigClaw();

        // Close the small claw.
        robotHardware.closeSmallClaw();

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

    // Gets the main action.
    private Action getMainAction() {
        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        TrajectoryActionBuilder driveToChamber1Builder = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(START_X, CHAMBER_Y));
        Action driveToChamber1 = driveToChamber1Builder.build();
        Action mainAction = new SequentialAction(
                driveToChamber1
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

}