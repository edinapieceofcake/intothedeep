package edu.edina.OpModes.Autonomous;

import static edu.edina.OpModes.Autonomous.AutoSample.TIMEOUT_MILLISECONDS;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
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

		// Initialize the wrist.
		robotHardware.initializeWrist();

		// Reset the swivel.
		robotHardware.swivelSetHorizontal();

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
/*
		// Construct a chamber pose.
		Pose2d chamberPose2 = new Pose2d(CHAMBER_X + CHAMBER_X_INCREMENT * 2, CHAMBER_Y, CHAMBER_HEADING);
		Pose2d scorePose2 = new Pose2d(SCORE_X + CHAMBER_X_INCREMENT * 2, SCORE_Y, CHAMBER_HEADING);
		Pose2d chamberPose3 = new Pose2d(SCORE_X + CHAMBER_X_INCREMENT * 3, CHAMBER_Y, CHAMBER_HEADING);
		Pose2d scorePose3 = new Pose2d(SCORE_X + CHAMBER_X_INCREMENT * 3, SCORE_Y, CHAMBER_HEADING);
		Pose2d chamberPose4 = new Pose2d(SCORE_X + CHAMBER_X_INCREMENT * 4, CHAMBER_Y, CHAMBER_HEADING);
		Pose2d scorePose4 = new Pose2d(SCORE_X + CHAMBER_X_INCREMENT * 4, SCORE_Y, CHAMBER_HEADING);

		// Construct a constant pose.
		Pose2d constantPose = new Pose2d(CONSTANT_X, CONSTANT_Y, CHAMBER_HEADING);
		// Construct a human player 2 pose.
		Pose2d humanPlayer2Pose = new Pose2d(HUMAN_PAYER_2_X, HUMAN_PAYER_2_Y, HUMAN_PLAYER_HEADING);

		// Construct a third spike mark pose.
		Pose2d thirdSpikeMarkPose = new Pose2d(THIRD_SPIKE_MARK_X, THIRD_SPIKE_MARK_Y, FIRST_SPIKE_MARK_HEADING);
		// Construct a human player pose.
		Pose2d humanPlayerPose = new Pose2d(HUMAN_PLAYER_X, HUMAN_PLAYER_Y, HUMAN_PLAYER_HEADING);
*/
		// Construct a start pose.
		Pose2d startPose = new Pose2d(0, -62.5, Math.toRadians(180));

		// Construct a drive interface.
		MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

		// Construct an action for driving from the start to the chamber position.
		TrajectoryActionBuilder driveFromStartToChamberBuilder = drive.actionBuilder(startPose)
				.strafeToLinearHeading(new Vector2d(0, -35), Math.toRadians(270));
		Action driveFromStartToChamber = driveFromStartToChamberBuilder.build();

		// Construct an action for driving from the chamber to the score position.
		TrajectoryActionBuilder driveFromChamberToScoreBuilder = driveFromStartToChamberBuilder.endTrajectory().fresh()
				.strafeToLinearHeading(new Vector2d(0, -42), Math.toRadians(270));
		Action driveFromChamberToScore = driveFromChamberToScoreBuilder.build();

/*
		driveFromChamberToScore2 = drive.actionBuilder(chamberPose2)
				.strafeToLinearHeading(scorePose2.position, scorePose2.heading)
				.build();

		driveFromChamberToScore3 = drive.actionBuilder(chamberPose3)
				.strafeToLinearHeading(scorePose3.position, scorePose3.heading)
				.build();

		driveFromChamberToScore4 = drive.actionBuilder(chamberPose4)
				.strafeToLinearHeading(scorePose4.position, scorePose4.heading)
				.build();

		// Construct an action for driving from the score to spike mark.
		Action driveFromScoreToThirdSpikeMark = drive.actionBuilder(scorePose)
				.strafeToLinearHeading(thirdSpikeMarkPose.position, thirdSpikeMarkPose.heading)
				.build();
		// Construct an action for driving from the third spike mark to human player.
		Action driveFromThirdSpikeMarkToHumanPlayer = drive.actionBuilder(thirdSpikeMarkPose)
				.strafeToLinearHeading(humanPlayerPose.position, humanPlayerPose.heading)
				.build();
		// Construct an action for driving from the human player to chamber.
		Action driveFromHumanPlayerToChamber = drive.actionBuilder(humanPlayerPose)
				.strafeToLinearHeading(chamberPose.position, chamberPose.heading)
				.build();
		// Construct an action for driving from the score to human player.
		Action driveFromScoreToHumanPlayer = drive.actionBuilder(scorePose)
				.strafeToLinearHeading(humanPlayerPose.position, humanPlayerPose.heading)
				.build();
		// Construct an action for driving from the human player to constant position.
		Action driveFromHHumanPlayerToConstant = drive.actionBuilder(humanPlayerPose)
				.strafeToLinearHeading(constantPose.position, constantPose.heading)
				.build();
		// Construct an action for driving from the constant position to the 2nd human player position.
		Action driveFromConstantToHumanPlayer2 = drive.actionBuilder(constantPose)
				.strafeToLinearHeading(humanPlayer2Pose.position, humanPlayer2Pose.heading)
				.build();
		// Construct an action for driving from the 2nd human player position to the chamber.
		Action driveFromHumanPlayer2ToChamber = drive.actionBuilder(humanPlayer2Pose)
				.strafeToLinearHeading(chamberPose.position, chamberPose.heading)
				.build();
		*/
			/*Action testDriveFromHumanPlayerToChamberPose3 = drive.actionBuilder(humanPlayer2Pose)
					.strafeToLinearHeading(chamberPose3.position, chamberPose3.heading)
					.build();*/
			/*driveFromChamberToScore3 = drive.actionBuilder(chamberPose3)
					.strafeToLinearHeading(scorePose3.position, scorePose3.heading)
					.build();*/

		// Construct velocity constraint.
		TranslationalVelConstraint velocityConstraint = new TranslationalVelConstraint(30);

		// Construct plow first sample action.
		TrajectoryActionBuilder plowFirstSampleBuilder = driveFromChamberToScoreBuilder.endTrajectory().fresh()
				.setTangent(Math.toRadians(0))
				.splineToConstantHeading(new Vector2d(36, -24), Math.toRadians(90), velocityConstraint)
				.splineToConstantHeading(new Vector2d(42, -16), Math.toRadians(0), velocityConstraint)
				.splineToConstantHeading(new Vector2d(48, -49), Math.toRadians(270), velocityConstraint);
		Action plowFirstSample = plowFirstSampleBuilder.build();

		// Construct plow second sample action.
		TrajectoryActionBuilder plowSecondSampleBuilder = plowFirstSampleBuilder.endTrajectory().fresh()
				.setTangent(Math.toRadians(90))
				.splineToConstantHeading(new Vector2d(48, -13), Math.toRadians(0), velocityConstraint)
				.splineToConstantHeading(new Vector2d(50, -13), Math.toRadians(0), velocityConstraint)
				.splineToConstantHeading(new Vector2d(54, -49), Math.toRadians(270), velocityConstraint);
		Action plowSecondSample = plowSecondSampleBuilder.build();

		// Construct a drive to first wall specimen action.
		TrajectoryActionBuilder driveToFirstWallSpecimenBuilder = plowSecondSampleBuilder.endTrajectory().fresh()
				.setTangent(Math.toRadians(90))
				.lineToY(-54);
		Action driveToFirstWallSpecimen = driveToFirstWallSpecimenBuilder.build();

		//Pose2d spikeMark1 = new Pose2d(46,-12, Math.toRadians(272));
		//Pose2d humanPlayer1 = new Pose2d(48, -55, Math.toRadians(272));
		//double humanPlayer1Tangent = Math.toRadians(270);
			/*
			Action driveToHumanPlayer1 = drive.actionBuilder(spikeMark1B)
					.setTangent(Math.toRadians(270))
					.splineToLinearHeading((humanPlayer1), humanPlayer1Tangent)
					.build();
			Action driveToSpikeMark1B = drive.actionBuilder(humanPlayer1)
					.strafeToLinearHeading(spikeMark1B.position, spikeMark1B.heading)
					.build();
			Pose2d spikeMark2 = new Pose2d(57, -10, Math.toRadians(272));
			Action driveToSpikeMark2A = drive.actionBuilder(spikeMark1B)
					.strafeToLinearHeading(spikeMark2.position, spikeMark2.heading)
					.build();
			Pose2d humanPlayer2 = new Pose2d(57, -55, Math.toRadians(272));
			Action driveToHumanPlayer2 = drive.actionBuilder(spikeMark2)
					.strafeToLinearHeading(humanPlayer2.position, humanPlayer2.heading)
					.build();
			Action driveToSpikeMark2B = drive.actionBuilder(humanPlayer2)
					.strafeToLinearHeading(spikeMark2.position, spikeMark2.heading)
					.build();
			Pose2d spikeMark3 = new Pose2d(61, -10, Math.toRadians(270));
			Action driveToSpikeMark3A = drive.actionBuilder(spikeMark2)
					.strafeToLinearHeading(spikeMark3.position, spikeMark3.heading)
					.build();
			Pose2d humanPlayer3 = new Pose2d(61, -55, Math.toRadians(270));
			Action driveToHumanPlayer3 = drive.actionBuilder(spikeMark3)
					.strafeToLinearHeading(humanPlayer3.position, humanPlayer3.heading)
					.build();
			Pose2d humanPlayerTransSition1 = new Pose2d(61,-50,Math.toRadians(270));
			Action driveToTransition1 = drive.actionBuilder(humanPlayer3)
					.strafeToLinearHeading(humanPlayerTransSition1.position, humanPlayerTransSition1.heading)
					.build();
			Pose2d humanPlayerTransSition2 = new Pose2d(55,-50,Math.toRadians(270));
			Action driveToTransition2 = drive.actionBuilder(humanPlayerTransSition1)
					.strafeToLinearHeading(humanPlayerTransSition2.position, humanPlayerTransSition2.heading)
					.build();
			Pose2d humanPlayerPickup = new Pose2d(55,-55,Math.toRadians(270));
			Action driveTohumanPlayerPickup = drive.actionBuilder(humanPlayerTransSition2)
					.strafeToLinearHeading(humanPlayerPickup.position, humanPlayerPickup.heading)
					.build();
			Pose2d firstScore = new Pose2d(0,-35,Math.toRadians(90));
			Action driveTofirstScore = drive.actionBuilder(humanPlayerPickup)
					.strafeToLinearHeading(firstScore.position, firstScore.heading)
					.build();
			Action driveFromFirstScoreToHumanPlayerPickup = drive.actionBuilder(firstScore)
					.strafeToLinearHeading(humanPlayerPickup.position, humanPlayerPickup.heading)
					.build();
			Pose2d secondScore = new Pose2d(2,-35,Math.toRadians(90));
			Action driveToSecondScore = drive.actionBuilder(humanPlayerPickup)
					.strafeToLinearHeading(secondScore.position, secondScore.heading)
					.build();
			Action driveFromSecondScoreToHumanPlayerPickup = drive.actionBuilder(secondScore)
					.strafeToLinearHeading(humanPlayerPickup.position, humanPlayerPickup.heading)
					.build();
			Pose2d thirdScore = new Pose2d(4,-35,Math.toRadians(90));
			Action driveToThirdScore = drive.actionBuilder(humanPlayerPickup)
					.strafeToLinearHeading(thirdScore.position, thirdScore.heading)
					.build();
			Action driveFromThirdScoreToHumanPlayerPickup = drive.actionBuilder(thirdScore)
					.strafeToLinearHeading(humanPlayerPickup.position, humanPlayerPickup.heading)
					.build();
			Pose2d fourthScore = new Pose2d(6,-35,Math.toRadians(90));
			Action driveToFourthScore = drive.actionBuilder(humanPlayerPickup)
					.strafeToLinearHeading(fourthScore.position, fourthScore.heading)
					.build();
			Action driveFromFourthScoreToHumanPlayerPickup = drive.actionBuilder(fourthScore)
					.strafeToLinearHeading(humanPlayerPickup.position, humanPlayerPickup.heading)
					.build();
			Pose2d fithScore = new Pose2d(8,-35,Math.toRadians(90));
			Action driveToFithScore = drive.actionBuilder(humanPlayerPickup)
					.strafeToLinearHeading(fithScore.position, fithScore.heading)
					.build();
			Action driveFromFithScoreToHumanPlayerPickup = drive.actionBuilder(fithScore)
					.strafeToLinearHeading(humanPlayerPickup.position, humanPlayerPickup.heading)
					.build();
			*/


		// Construct a main action.
		Action mainAction = new SequentialAction(
				new InstantAction(() -> robotHardware.raiseWrist()),
				// Score preloaded specimen.
				driveFromStartToChamber,
				scoreSpecimenAndLower(driveFromChamberToScore),
				plowFirstSample,
				plowSecondSample,
				driveToFirstWallSpecimen,
				new InstantAction(() -> robotHardware.closeClaw())
				//driveToSpikeMark1,
				// deliver at human player
					/*driveToHumanPlayer1,
					driveToSpikeMark1B,
					driveToSpikeMark2A,
					// deliver at human player
					goToWallPosition(),
					driveToHumanPlayer2,*/
					/*driveToSpikeMark2B,
					driveToSpikeMark3A,
					// deliver at human player
					driveToHumanPlayer3,
					driveToTransition1,
					driveToTransition2,*/
				// grab the specimen from wall
					/*driveTohumanPlayerPickup,
					new InstantAction(() -> robotHardware.closeClaw()),
					driveTofirstScore,
					// score the 2nd specimen
					scoreSpecimenAndLower(driveFromChamberToScore),
					goToWallPosition(),
					driveFromFirstScoreToHumanPlayerPickup,
					new InstantAction(() -> robotHardware.closeClaw()),
					// grab the specimen from wall
					driveToSecondScore,
				//	driveToSecondScore,
					// score the 3rd specimen
					scoreSpecimenAndLower(driveFromChamberToScore2),
					goToWallPosition(),
					driveFromSecondScoreToHumanPlayerPickup,
					new InstantAction(() -> robotHardware.closeClaw()),
					// grab the specimen from wall
					driveToThirdScore,
					// score the 4th specimen
					scoreSpecimenAndLower(driveFromChamberToScore3),
					goToWallPosition(),
					driveFromThirdScoreToHumanPlayerPickup,
					new InstantAction(() -> robotHardware.closeClaw()),
					// grab the specimen from wall
					driveToFourthScore,
					scoreSpecimenAndLower(driveFromChamberToScore4)
					// score the 5th specimen
					/*scoreSpecimenAndLower(driveFromChamberToScore),
					// Park
					driveFromFourthScoreToHumanPlayerPickup*/
//					driveToFithScore,
//					driveFromFithScoreToHumanPlayerPickup*/



					/*testDriveToChamberFromHumanPlayer1,
					scoreSpecimenAndLower(driveFromChamberToScore2),
					testDriveFromScoreToHumanPlayer,
					new InstantAction(() -> robotHardware.closeClaw()),
					new WaitForTime(CLAW_DELAY),
					testDriveFromHumanPlayerToChamberPose3,
					scoreSpecimenAndLower(driveFromChamberToScore3)*/




					/*
					// Grab first spike mark sample.
					driveFromScoreToFirstSpikeMark,
					new InstantAction(() -> robotHardware.closeClaw()),
					new WaitForTime(CLAW_DELAY),

					// Deliver first spike mark sample.
					deliverSampleToHumanPlayer(),

					// Grab second spike mark sample.
					driveFromFirstSpikeMarkToSecondSpikeMark,
					new InstantAction(() -> robotHardware.closeClaw()),
					new WaitForTime(CLAW_DELAY),

					// Deliver second spike mark sample.
					deliverSampleToHumanPlayer()
					 */

//							//deliver sample to human player
//							driveFirstSpikeMarkToHumanPlayer,
//							new OpenClaw(),
//							//pick up new sample from 2nd spike mark
//							driveFromHumanToSecondSpikeMark,
//							new CloseClaw(),
//							//deliver to sample human player
//							driveFromSecondSpikeMarkToHumanPlayer,
//							new OpenClaw(),
//							//align with the specimen from the first spike mark
//							driveFromHHumanPlayerToConstant,
//							driveFromConstantToHumanPlayer2,
//							new CloseClaw(),
//							// score specimen
//							driveFromHumanPlayer2ToChamber,
//							score(),
//							//pick up sample from 3rd spike
//							driveFromScoreToThirdSpikeMark,
//							new CloseClaw(),
//							// deliver to human player
//							driveFromThirdSpikeMarkToHumanPlayer,
//							new OpenClaw(),
//							//align with specimen from 2nd spike mark
//							driveFromHHumanPlayerToConstant,
//							driveFromConstantToHumanPlayer2,
//							new CloseClaw(),
//							//score the specimen from the 2nd spike mark, but third sample total.
//							driveFromHumanPlayer2ToChamber,
//							score(),
//							// pick up specimen that was 3rd spike mark from the human player
//							driveFromScoreToHumanPlayer,
//							new CloseClaw(),
//							// score the specimen
//							driveFromHumanPlayerToChamber,
//							score()
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

	// Scores a specimen and then lowers the arm.
	private Action scoreSpecimenAndLower(Action driveFromChamberToScore) {

		// Construct an action.
		Action action = new SequentialAction(
				new InstantAction(() -> robotHardware.setWristHighChamberHoldPosition()),
				new InstantAction(() -> robotHardware.swivelSetClip()),
				new MoveArm(robotHardware, Arm.HIGH_CHAMBER_POSITION, false),
				new WaitForTime(500),
				scoreSpecimen(driveFromChamberToScore, robotHardware),
				new WaitForTime(500),
				new MoveArm(robotHardware, Arm.WALL_POSITION, false),
				new WaitForHardware(robotHardware, TIMEOUT_MILLISECONDS),
				new InstantAction(() -> robotHardware.setWristWallPosition())
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
	/*
	// Grab the Speciemn from the human player
	public Action goToWallPosition() {
		// ONLY USE THIS FOR AUTO, IT DOES NOT CLEAR ACTIONS
		Action action = new ParallelAction(
				new MoveArm(robotHardware, Arm.WALL_POSITION, true),
				new InstantAction(() -> robotHardware.setWristWallPosition()),
				new InstantAction(() -> robotHardware.swivelSetHorizontal()),
				new InstantAction(() -> robotHardware.setLiftGroundPosition()),
				new InstantAction(() -> robotHardware.setMinimumExtension())
		);
		return action;
	}
	*/
	/*
	// Delivers a sample to a human player.
	public Action deliverSampleToHumanPlayer() {

		// Construct an action.
		Action action = new SequentialAction(
				new MoveArm(robotHardware, Arm.SUBMERSIBLE_POSITION, false),
				new WaitForHardware(robotHardware, TIMEOUT_MILLISECONDS),
				new InstantAction(() -> robotHardware.openClaw()),
				new WaitForTime(CLAW_DELAY),
				new MoveArm(robotHardware, Arm.GROUND_POSITION, false),
				new WaitForHardware(robotHardware, TIMEOUT_MILLISECONDS)
		);

		// Return the action.
		return action;

	}
	*/
}