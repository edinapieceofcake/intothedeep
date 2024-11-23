package edu.edina.OpModes.Autonomous;

import static edu.edina.Libraries.Robot.RobotHardware.SCORE_SPECIMEN_BACKUP_INCHES;
import static edu.edina.OpModes.Autonomous.AutoSample.TIMEOUT_MILLISECONDS;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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

		// Start pose
		public static double START_X = 0;
		public static double START_Y = -61.5;
		public static double START_HEADING = Math.toRadians(270);

		// Chamber pose
		public static double CHAMBER_X = 0;
		public static double CHAMBER_X_INCREMENT = 3;
		public static double CHAMBER_Y = -35;
		public static double CHAMBER_HEADING = START_HEADING;

		// Score pose
		public static double SCORE_X = CHAMBER_X;
		public static double SCORE_Y = CHAMBER_Y - SCORE_SPECIMEN_BACKUP_INCHES;

		// First spike mark pose
		public static double FIRST_SPIKE_MARK_A_X = 49;
		public static double FIRST_SPIKE_MARK_A_Y = -47;
		public static double FIRST_SPIKE_MARK_B_X = FIRST_SPIKE_MARK_A_X;
		public static double FIRST_SPIKE_MARK_B_Y = -41;
		public static double FIRST_SPIKE_MARK_HEADING = Math.toRadians(90);

		// Second spike mark pose
		public static double SECOND_SPIKE_MARK_A_X = 59;
		public static double SECOND_SPIKE_MARK_A_Y = FIRST_SPIKE_MARK_A_Y;
		public static double SECOND_SPIKE_MARK_B_X = SECOND_SPIKE_MARK_A_X;
		public static double SECOND_SPIKE_MARK_B_Y = FIRST_SPIKE_MARK_B_Y;
		public static double SECOND_SPIKE_MARK_HEADING = Math.toRadians(90);

		// Third Spike Mark pose
		public static double THIRD_SPIKE_MARK_X = 58;
		public static double THIRD_SPIKE_MARK_Y = -25;
		public static double HUMAN_PLAYER_X = 47;
		public static double HUMAN_PLAYER_Y = -50;
		public static double CONSTANT_Y = -46.5;
		public static double CONSTANT_X = 47;
		public static double HUMAN_PAYER_2_Y = -50;
		public static double HUMAN_PAYER_2_X = 44;
		public static double HUMAN_PLAYER_HEADING = 3.0 / 2 * Math.PI;
		// Duration in milliseconds to toggle the claw
		public static int CLAW_DELAY = 500;
		public static int ARM_DELAY = 500;
		public static int SCORE_DELAY = 400;


		// Robot hardware
		private RobotHardware robotHardware;
		private Action driveFromChamberToScore;
		private Action driveFromChamberToScore2;

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

			// Construct a chamber pose.
			Pose2d chamberPose = new Pose2d(CHAMBER_X, CHAMBER_Y, CHAMBER_HEADING);
			Pose2d scorePose = new Pose2d(SCORE_X, SCORE_Y, CHAMBER_HEADING);

			// Construct a chamber pose.
			Pose2d chamberPose2 = new Pose2d(CHAMBER_X + CHAMBER_X_INCREMENT, CHAMBER_Y, CHAMBER_HEADING);
			Pose2d scorePose2 = new Pose2d(SCORE_X + CHAMBER_X_INCREMENT, SCORE_Y, CHAMBER_HEADING);

			// Construct a first spike mark pose.
			Vector2d firstSpikeMarkAVector = new Vector2d(FIRST_SPIKE_MARK_A_X, FIRST_SPIKE_MARK_A_Y);
			Vector2d firstSpikeMarkBVector = new Vector2d(FIRST_SPIKE_MARK_B_X, FIRST_SPIKE_MARK_B_Y);
			Pose2d firstSpikeMarkPose = new Pose2d(FIRST_SPIKE_MARK_B_X, FIRST_SPIKE_MARK_B_Y, FIRST_SPIKE_MARK_HEADING);

			// Construct a second spike mark pose.
			Vector2d secondSpikeMarkAVector = new Vector2d(SECOND_SPIKE_MARK_A_X, SECOND_SPIKE_MARK_A_Y);
			Vector2d secondSpikeMarkBVector = new Vector2d(SECOND_SPIKE_MARK_B_X, SECOND_SPIKE_MARK_B_Y);
			Pose2d secondSpikeMarkPose = new Pose2d(SECOND_SPIKE_MARK_B_X, SECOND_SPIKE_MARK_B_Y, SECOND_SPIKE_MARK_HEADING);

			// Construct a constant pose.
			Pose2d constantPose = new Pose2d(CONSTANT_X, CONSTANT_Y, CHAMBER_HEADING);
			// Construct a human player 2 pose.
			Pose2d humanPlayer2Pose = new Pose2d(HUMAN_PAYER_2_X, HUMAN_PAYER_2_Y, HUMAN_PLAYER_HEADING);

			// Construct a third spike mark pose.
			Pose2d thirdSpikeMarkPose = new Pose2d(THIRD_SPIKE_MARK_X, THIRD_SPIKE_MARK_Y, FIRST_SPIKE_MARK_HEADING);
			// Construct a human player pose.
			Pose2d humanPlayerPose = new Pose2d(HUMAN_PLAYER_X, HUMAN_PLAYER_Y, HUMAN_PLAYER_HEADING);

			// Testing
			double testSubToSpikeMarkTangent = 15.0/8*Math.PI;
			Pose2d testFirstSpikeMarkPose = new Pose2d(47,-35,Math.toRadians(89));
			double testFirstSpikeMarkTangent = Math.toRadians(89);
			Vector2d testDropFirstSpikeMarkSampleVector = new Vector2d(53, -44);
			double testDropFirstSpikeMarkSampleHeading = 3.0/2*Math.PI;
			Vector2d testGrabFirstSpecimenFromHumanPlayerVector = new Vector2d(47, -44);
			double testGrabFirstSpecimenFromHumanPlayerHeading = 3.0/2*Math.PI;

			// Construct a drive interface.
			MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

			// Construct an action for driving from the start to the chamber.
			Action driveFromStartToChamber = drive.actionBuilder(startPose)
					.strafeToLinearHeading(chamberPose.position, chamberPose.heading)
					.build();
			driveFromChamberToScore = drive.actionBuilder(chamberPose)
					.strafeToLinearHeading(scorePose.position, scorePose.heading)
					.build();

			// Construct an action for driving from the score to the first spike mark.
			Action driveFromScoreToFirstSpikeMark = drive.actionBuilder(scorePose)
					.strafeToLinearHeading(firstSpikeMarkAVector, FIRST_SPIKE_MARK_HEADING)
					.strafeTo(firstSpikeMarkBVector)
					.build();

			// Construct an action for driving from the first spike mark to the second spike mark.
			Action driveFromFirstSpikeMarkToSecondSpikeMark = drive.actionBuilder(firstSpikeMarkPose)
					.strafeToLinearHeading(secondSpikeMarkAVector, SECOND_SPIKE_MARK_HEADING)
					.strafeTo(secondSpikeMarkBVector)
					.build();


			// Construct an action for driving from the first spike mark to human player.
			Action driveFirstSpikeMarkToHumanPlayer = drive.actionBuilder(firstSpikeMarkPose)
					.strafeToLinearHeading(humanPlayerPose.position, humanPlayerPose.heading)
					.build();

			// Construct an action for driving from the human player to second spike mark.
			Action driveFromHumanToSecondSpikeMark = drive.actionBuilder(humanPlayerPose)
					.strafeToLinearHeading(secondSpikeMarkPose.position, secondSpikeMarkPose.heading)
					.build();

			// Construct an action for driving from the second spike mark to the human player.
			Action driveFromSecondSpikeMarkToHumanPlayer = drive.actionBuilder(secondSpikeMarkPose)
					.strafeToLinearHeading(humanPlayerPose.position, humanPlayerPose.heading)
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

			// Testing
			Action testSetTangentForDrivingFromSubToSpikeMark = drive.actionBuilder(scorePose)
					.setTangent(testSubToSpikeMarkTangent)
					.build();

			Action testDriveFromSubToFirstSpikeMark = drive.actionBuilder(new Pose2d(scorePose.position, testSubToSpikeMarkTangent))
					.splineToLinearHeading(testFirstSpikeMarkPose, testFirstSpikeMarkTangent)
					.build();

			Action testSetTangentToNormal = drive.actionBuilder(scorePose)
					.setTangent(testSubToSpikeMarkTangent)
					.build();

			Action testDriveFromFirstSpikeMarkToHumanPlayer = drive.actionBuilder(testFirstSpikeMarkPose)
					.strafeToLinearHeading(testDropFirstSpikeMarkSampleVector, testDropFirstSpikeMarkSampleHeading)
					.build();

			Action testDriveToFirstHumanPlayerSpecimen = drive.actionBuilder(new Pose2d(testDropFirstSpikeMarkSampleVector, testDropFirstSpikeMarkSampleHeading))
					.strafeToLinearHeading(testGrabFirstSpecimenFromHumanPlayerVector, testGrabFirstSpecimenFromHumanPlayerHeading)
					.build();

			Action testDriveToChamberFromHumanPlayer1 = drive.actionBuilder(new Pose2d(testGrabFirstSpecimenFromHumanPlayerVector, testGrabFirstSpecimenFromHumanPlayerHeading))
					.strafeToLinearHeading(chamberPose2.position, chamberPose2.heading)
					.build();
			driveFromChamberToScore2 = drive.actionBuilder(chamberPose2)
					.strafeToLinearHeading(scorePose2.position, scorePose2.heading)
					.build();

			// Construct a main action.
			Action mainAction = new SequentialAction(

					// Score preloaded specimen.
					driveFromStartToChamber,
					scoreSpecimenAndLower(driveFromChamberToScore),

					// Test
					testSetTangentForDrivingFromSubToSpikeMark,
					testDriveFromSubToFirstSpikeMark,
					testSetTangentToNormal,
					new InstantAction(() -> robotHardware.closeClaw()),
					new WaitForTime(CLAW_DELAY),
					new InstantAction(() -> robotHardware.incrementArmPosition()),
					new InstantAction(() -> robotHardware.incrementArmPosition()),
					new InstantAction(() -> robotHardware.incrementArmPosition()),
					new WaitForTime(ARM_DELAY),
					testDriveFromFirstSpikeMarkToHumanPlayer,
					new InstantAction(() -> robotHardware.decrementArmPosition()),
					new InstantAction(() -> robotHardware.decrementArmPosition()),
					new InstantAction(() -> robotHardware.decrementArmPosition()),
					new WaitForTime(ARM_DELAY),
					new InstantAction(() -> robotHardware.openClaw()),
					new WaitForTime(CLAW_DELAY),
					new InstantAction(() -> robotHardware.incrementArmPosition()),
					new InstantAction(() -> robotHardware.incrementArmPosition()),
					new InstantAction(() -> robotHardware.incrementArmPosition()),
					new WaitForTime(ARM_DELAY),
					testDriveToFirstHumanPlayerSpecimen,
					new InstantAction(() -> robotHardware.decrementArmPosition()),
					new InstantAction(() -> robotHardware.decrementArmPosition()),
					new InstantAction(() -> robotHardware.decrementArmPosition()),
					new WaitForTime(ARM_DELAY),
					new InstantAction(() -> robotHardware.closeClaw()),
					new WaitForTime(CLAW_DELAY),
					testDriveToChamberFromHumanPlayer1,
					scoreSpecimenAndLower(driveFromChamberToScore2)

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
        public Action scoreSpecimenAndLower(Action driveFromChamberToScore) {

			// Construct an action.
            Action action = new SequentialAction(
					new InstantAction(() -> robotHardware.lowerWrist()),
					new MoveArm(robotHardware, Arm.HIGH_CHAMBER_POSITION, false),
					new WaitForTime(500),
					scoreSpecimen(driveFromChamberToScore, robotHardware),
					new WaitForTime(500),
					new MoveArm(robotHardware, Arm.GROUND_POSITION, false),
                    new WaitForHardware(robotHardware, TIMEOUT_MILLISECONDS),
					new InstantAction(() -> robotHardware.lowerWrist())
            );

			// Return the action.
			return action;

        }

		// Scores a specimen.
		public static Action scoreSpecimen(Action backup, RobotHardware robotHardware) {

			// Construct a score specimen action.
			Action action = new ParallelAction(
					new InstantAction(() -> robotHardware.moveWristToHighChamberScore()),
					backup,
					new SequentialAction(
							new WaitForTime(SCORE_DELAY),
							new InstantAction(() -> robotHardware.openClaw())
					)
			);

			// Return the action.
			return action;

		}

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

	}