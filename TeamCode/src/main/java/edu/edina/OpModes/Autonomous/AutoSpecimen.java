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

	// Slow velocity
	public static double SLOW_VELOCITY = 15;

	// Fast velocity
	public static double FAST_VELOCITY = 30;

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

		// Close the claw.
		robotHardware.closeClaw();

		// Initialize the wrist.
		robotHardware.initializeWrist();

		// Reset the swivel.
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

		// Construct velocity constraints.
		//////////////////////////////////////////////////////////////////////

		// Construct a plow velocity constraint.
		TranslationalVelConstraint plowVelocityConstraint = new TranslationalVelConstraint(30);

		// Construct a wall velocity constraint.
		VelConstraint wallVelocityConstraint = (robotPose, _path, _disp) -> {

			// Determine whether the robot is close to the chamber.
			boolean closeToChamber = isCloseToChamber(robotPose);

			// Determine whether the robot is close to the wall.
			boolean closeToWall = isCloseToWall(robotPose);

			// If the robot is close to the chamber or wall...
			if (closeToChamber || closeToWall) {

				// Go slow.
				return SLOW_VELOCITY;
			}

			// Otherwise (if the robot is far from the chamber and wall)...
			else {

				// Go fast.
				return FAST_VELOCITY;

			}

		};

		// Construct a chamber velocity constraint.
		VelConstraint chamberVelocityConstraint = (robotPose, _path, _disp) -> {

			// Determine whether the robot is close to the chamber.
			boolean closeToChamber = isCloseToChamber(robotPose);

			// If the robot is close to the chamber...
			if (closeToChamber) {

				// Go slow.
				return SLOW_VELOCITY;

			}

			// Otherwise (if the robot is far from the chamber)...
			else {

				// Go fast.
				return FAST_VELOCITY;

			}

		};

		// Construct trajectories.
		//////////////////////////////////////////////////////////////////////

		// Construct a start pose.
		Pose2d startPose = new Pose2d(0, -62.5, Math.toRadians(180));

		// Construct a drive interface.
		MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

		// Construct a first drive to chamber action.
		TrajectoryActionBuilder driveToChamber1Builder = drive.actionBuilder(startPose)
				.strafeToLinearHeading(new Vector2d(0, -35), Math.toRadians(270));
		Action driveToChamber1 = driveToChamber1Builder.build();

		// Construct first plow action.
		TrajectoryActionBuilder plow1Builder = driveToChamber1Builder.endTrajectory().fresh()
				.setTangent(Math.toRadians(0))
				.splineToConstantHeading(new Vector2d(36, -24), Math.toRadians(90), plowVelocityConstraint)
				.splineToConstantHeading(new Vector2d(42, -14), Math.toRadians(0), plowVelocityConstraint)
				.splineToConstantHeading(new Vector2d(50, -49), Math.toRadians(270), plowVelocityConstraint);
		Action plow1 = plow1Builder.build();

		// Construct second plow action.
		TrajectoryActionBuilder plow2Builder = plow1Builder.endTrajectory().fresh()
				.setTangent(Math.toRadians(90))
				.splineToConstantHeading(new Vector2d(48, -13), Math.toRadians(0), plowVelocityConstraint)
				.splineToConstantHeading(new Vector2d(50, -13), Math.toRadians(0), plowVelocityConstraint)
				.splineToConstantHeading(new Vector2d(54, -49), Math.toRadians(270), plowVelocityConstraint);
		Action plow2 = plow2Builder.build();

		// Construct a first drive to wall action.
		TrajectoryActionBuilder driveToWall1Builder = plow2Builder.endTrajectory().fresh()
				.lineToY(-54, wallVelocityConstraint);
		Action driveToWall1 = driveToWall1Builder.build();

		// Construct a second drive to chamber action.
		TrajectoryActionBuilder driveToChamber2Builder = driveToWall1Builder.endTrajectory().fresh()
				.setTangent(Math.toRadians(90))
				.splineToConstantHeading(new Vector2d(6, -35), Math.toRadians(90), chamberVelocityConstraint);
		Action driveToChamber2 = driveToChamber2Builder.build();

		// Construct a second drive to second wall action.
		TrajectoryActionBuilder driveToWall2Builder = driveToChamber2Builder.endTrajectory().fresh()
				.setTangent(Math.toRadians(270))
				.splineToConstantHeading(new Vector2d(36, -54), Math.toRadians(270), wallVelocityConstraint);
		Action driveToWall2 = driveToWall2Builder.build();

		// Construct a third drive to chamber action.
		TrajectoryActionBuilder driveToChamber3Builder = driveToWall2Builder.endTrajectory().fresh()
				.setTangent(Math.toRadians(90))
				.splineToConstantHeading(new Vector2d(3, -35), Math.toRadians(90), chamberVelocityConstraint);
		Action driveToChamber3 = driveToChamber3Builder.build();

		// Construct a third drive to wall action.
		TrajectoryActionBuilder driveToWall3Builder = driveToChamber3Builder.endTrajectory().fresh()
				.setTangent(Math.toRadians(270))
				.splineToConstantHeading(new Vector2d(36, -54), Math.toRadians(270), wallVelocityConstraint);
		Action driveToWall3 = driveToWall3Builder.build();

		// Construct a fourth drive to chamber action.
		TrajectoryActionBuilder driveToChamber4Builder = driveToWall3Builder.endTrajectory().fresh()
				.setTangent(Math.toRadians(90))
				.splineToConstantHeading(new Vector2d(0, -35), Math.toRadians(90), chamberVelocityConstraint);
		Action driveToChamber4 = driveToChamber4Builder.build();

		// Construct a fourth drive to score action.
		TrajectoryActionBuilder driveToScore4Builder = driveToChamber4Builder.endTrajectory().fresh()
				.strafeTo(new Vector2d(0, -42));
		Action driveToScore4 = driveToScore4Builder.build();

		// Construct a drive to park action.
		TrajectoryActionBuilder driveToParkBuilder = driveToScore4Builder.endTrajectory().fresh()
				.strafeTo(new Vector2d(54, -54));
		Action driveToPark = driveToParkBuilder.build();

		// Construct a main action.
		//////////////////////////////////////////////////////////////////////

		// Construct a main action.
		Action mainAction = new SequentialAction(

				// Drive to the chamber while raising the arm.
				new ParallelAction(
						driveToChamber1,
						raiseArmToChamber()
				),

				// Wait for the arm to settle.
				new WaitForTime(500),

				// Score the preloaded specimen.
				new ParallelAction(
						scoreSpecimen(plow1, robotHardware),
						new SequentialAction(
								new WaitForTime(1000),
								lowerArmFromChamber(true)
						)
				),

				// Plow the second sample.
				plow2,

				// Drive to the wall.
				driveToWall1,

				// Close the claw.
				new InstantAction(() -> robotHardware.closeClaw()),
				new WaitForTime(200),

				// Drive to the chamber while raising the arm.
				new ParallelAction(
						driveToChamber2,
						raiseArmToChamber()
				),

				// Score the first wall specimen.
				new ParallelAction(
						scoreSpecimen(driveToWall2, robotHardware),
						new SequentialAction(
								new WaitForTime(200),
								lowerArmFromChamber(true)
						),
						new SequentialAction(
								new WaitForTime(800),
								new InstantAction(() -> robotHardware.setWristWallPosition())
						)
				),

				// Close the claw.
				new InstantAction(() -> robotHardware.closeClaw()),
				new WaitForTime(200),

				// Drive to the chamber while raising the arm.
				new ParallelAction(
						driveToChamber3,
						raiseArmToChamber()
				),

				// Score the second wall specimen.
				new ParallelAction(
						scoreSpecimen(driveToWall3, robotHardware),
						new SequentialAction(
								new WaitForTime(200),
								lowerArmFromChamber(true)
						),
						new SequentialAction(
								new WaitForTime(800),
								new InstantAction(() -> robotHardware.setWristWallPosition())
						)
				),

				// Close the claw.
				new InstantAction(() -> robotHardware.closeClaw()),
				new WaitForTime(200),

				// Drive to the chamber while raising the arm.
				new ParallelAction(
						driveToChamber4,
						raiseArmToChamber()
				),

				// Score the third wall specimen.
				scoreSpecimen(driveToScore4, robotHardware),

				lowerArmFromChamber(false)

				// Drive to park while lowering the arm.
//				new ParallelAction(
//						driveToPark,
//						lowerArmFromChamber(false)
//				)

		);

		// Return the main action.
		return mainAction;

	}

	// Raises arm to chamber.
	private Action raiseArmToChamber() {

		// Construct an action.
		Action action = new ParallelAction(
				new InstantAction(() -> robotHardware.setWristHighChamberHoldPosition()),
				new SequentialAction(
						new WaitForTime(100),
						new InstantAction(() -> robotHardware.swivelSetClipNoDelay())
				),
				new MoveArm(robotHardware, Arm.HIGH_CHAMBER_POSITION, false)
		);

		// Return the action.
		return action;

	}

	// Lowers arm from chamber.
	private Action lowerArmFromChamber(boolean endAtWall) {

		// Construct an action.
		Action action = new SequentialAction(
				new WaitForTime(500),
				new InstantAction(() -> robotHardware.swivelSetHorizontal()),
				new InstantAction(endAtWall ? () -> robotHardware.setWristWallPosition() : () -> robotHardware.lowerWrist()),
				new MoveArm(robotHardware, endAtWall ? Arm.WALL_POSITION : Arm.GROUND_POSITION, false),
				new WaitForHardware(robotHardware, TIMEOUT_MILLISECONDS)
		);

		// Return the action.
		return action;

	}

	// Scores a specimen.
	public static Action scoreSpecimen(Action backup, RobotHardware robotHardware) {

		// Construct a score specimen action.
		Action action = new ParallelAction(
				new InstantAction(() -> robotHardware.setWristHighChamberClipPosition()),
				backup,
				new SequentialAction(
						new WaitForTime(400),
						new InstantAction(() -> robotHardware.openClaw())
				)
		);

		// Return the action.
		return action;

	}

	// Determines whether the robot is close to the chamber.
	private static boolean isCloseToChamber(Pose2dDual robotPose) {
		return robotPose.position.x.value() < 15 && robotPose.position.y.value() > -45;
	}

	// Determines whether the robot is close to the wall.
	private static boolean isCloseToWall(Pose2dDual robotPose) {
		return robotPose.position.x.value() > 28 && robotPose.position.y.value() < -45;
	}

}