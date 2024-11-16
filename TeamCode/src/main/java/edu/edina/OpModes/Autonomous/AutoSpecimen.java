package edu.edina.OpModes.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.Robot.RobotHardware;

@Config
@Autonomous(preselectTeleOp = "TeleOpMain")
public class AutoSpecimen extends LinearOpMode {

		// Start pose
		public static double START_X = 0;
		public static double START_Y = -61.5;
		public static double START_HEADING = 3.0/2*Math.PI;

		// Chamber Pose
		public static double CHAMBER_X = 0;
		public static double CHAMBER_Y = -35;
		public static double CHAMBER_HEADING = START_HEADING;

		// First spike mark pose
		public static double FIRST_SPIKE_MARK_X = 37.5;
		public static double FIRST_SPIKE_MARK_Y = -24.7;
		public static double SPIKE_MARK_HEADING = 4.0 / 2 * Math.PI;

		// Second spike mark pose
		public static double SECOND_SPIKE_MARK_X = 48;
		public static double SECOND_SPIKE_MARK_Y = -25;


		// Third Spike Mark pose
		public static double THIRD_SPIKE_MARK_X = 58;
		public static double THIRD_SPIKE_MARK_Y = -25;
		public static double HUMAN_PLAYER_X = 47;
		public static double HUMAN_PLAYER_Y = -50;
		public static double CONSTANT_Y = -46.5;
		public static double CONSTANT_X = 47;
		public static double HUMAN_PAYER_2_Y = -50;
		public static double HUMAN_PAYER_2_X = 44;
		public static double SCORE_Y = -42;
		public static double HUMAN_PLAYER_HEADING = 3.0 / 2 * Math.PI;
		// Duration in milliseconds to toggle the claw
		public static int CLAW_DELAY = 500;
		public static int SCORE_DELAY = 300;


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

			// Construct a start pose.
			Pose2d startPose = new Pose2d(START_X, START_Y, START_HEADING);

			// Construct a chamber pose.
			Pose2d chamberPose = new Pose2d(CHAMBER_X, CHAMBER_Y, CHAMBER_HEADING);
			Pose2d scorePose = new Pose2d(CHAMBER_X, SCORE_Y, CHAMBER_HEADING);

			// Construct a first spike mark pose.
			Pose2d firstSpikeMarkPose = new Pose2d(FIRST_SPIKE_MARK_X, FIRST_SPIKE_MARK_Y, SPIKE_MARK_HEADING);
			// Construct a constant pose.
			Pose2d constantPose = new Pose2d(CONSTANT_X, CONSTANT_Y, CHAMBER_HEADING);
			// Construct a human player 2 pose.
			Pose2d humanPlayer2Pose = new Pose2d(HUMAN_PAYER_2_X, HUMAN_PAYER_2_Y, HUMAN_PLAYER_HEADING);

			// Construct a second spike mark pose.
			Pose2d secondSpikeMarkPose = new Pose2d(SECOND_SPIKE_MARK_X, SECOND_SPIKE_MARK_Y, SPIKE_MARK_HEADING);
			// Construct a third spike mark pose.
			Pose2d thirdSpikeMarkPose = new Pose2d(THIRD_SPIKE_MARK_X, THIRD_SPIKE_MARK_Y, SPIKE_MARK_HEADING);
			// Construct a human player pose.
			Pose2d humanPlayerPose = new Pose2d(HUMAN_PLAYER_X, HUMAN_PLAYER_Y, HUMAN_PLAYER_HEADING);

			// Construct a drive interface.
			MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

			// Construct an action for driving from the start to the chamber.
			Action driveFromStartToChamber = drive.actionBuilder(startPose)
					.strafeToLinearHeading(chamberPose.position, chamberPose.heading)
					.build();
			Action driveFromChamberToScore = drive.actionBuilder(chamberPose)
					.strafeToLinearHeading(scorePose.position, scorePose.heading)
					.build();

			// Construct an action for driving from the score to the first spike mark.
			Action driveFromScoreToFirstSpikeMark = drive.actionBuilder(scorePose)
					.strafeToLinearHeading(firstSpikeMarkPose.position, firstSpikeMarkPose.heading)
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



			// Run the actions.
			Actions.runBlocking(
					new SequentialAction(
                            // score preloaded specimen

                            driveFromStartToChamber,
							getScoreAction(driveFromChamberToScore),
							//pick up sample on first spike mark
							driveFromScoreToFirstSpikeMark,
							new CloseClaw(),
							new WaitAndUpdate(CLAW_DELAY)
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
//								getScoreAction(driveFromChamberToScore)
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
//								getScoreAction(driveFromChamberToScore)
//							// pick up specimen that was 3rd spike mark from the human player
//							driveFromScoreToHumanPlayer,
//							new CloseClaw(),
//							// score the specimen
//							driveFromHumanPlayerToChamber,
//							getScoreAction(driveFromChamberToScore)
					)

			);

		}

        public Action getScoreAction(Action driveFromChamberToScore) {
            Action scoreAction = new SequentialAction(
                    new MoveToHighChamber(),
                    new WaitForNotBusy(),
					new WaitAndUpdate(200),
                    new ParallelAction(
                            new LowerWrist(),
                            driveFromChamberToScore,
                            new SequentialAction(
                                    new WaitAndUpdate(SCORE_DELAY),
                                    new OpenClaw()
                            )
                    ),
					new WaitAndUpdate(200),
                    new MoveToGround(),
                    new WaitForNotBusy()
            );
            return scoreAction;
        }

		// Opens the claw.
		public class OpenClaw implements Action {

			// Runs this.
			@Override
			public boolean run(@NonNull TelemetryPacket packet) {

				// Open the claw.
				robotHardware.openClaw();

				// Return indicating that the action is done.
				return false;

			}

		}

		// Closes the claw.
		public class CloseClaw implements Action {

			// Runs this.
			@Override
			public boolean run(@NonNull TelemetryPacket packet) {

				// Close the claw.
				robotHardware.closeClaw();

				// Return indicating that the action is done.
				return false;

			}

		}

		// Moves the claw to the high chamber.
		public class MoveToHighChamber implements Action {

			// Runs this.
			@Override
			public boolean run(@NonNull TelemetryPacket packet) {

				// Lower the wrist.
				robotHardware.lowerWrist();

				// Move the arm to the high chamber position.
				robotHardware.setArmHighChamberPosition();

				// Move the lift to the ground position
				robotHardware.setLiftGroundPosition();

				// Use the high chamber extension.
				robotHardware.setMinimumExtension();

				// Return indicating that the action is done.
				return false;

			}

		}

		// Moves the claw to the ground.
		public class MoveToGround implements Action {

			// Runs this.
			@Override
			public boolean run(@NonNull TelemetryPacket packet) {

				// Move the arm to the ground position.
				robotHardware.setArmGroundPosition();

				// Move the lift to the ground position
				robotHardware.setLiftGroundPosition();

				// Fully retract the slide.
				robotHardware.setMinimumExtension();

				// Return indicating that the action is done.
				return false;

			}

		}

		public class LowerWrist implements Action {

			// Runs this.
			@Override
			public boolean run(@NonNull TelemetryPacket packet) {
				// Move Wrist to High Chamber Score Position
				robotHardware.moveWristToHighChamberScorePosition();
				// Return indicating that the action is done.
				return false;

			}

		}

		// Waits for a specified duration.
		public class WaitAndUpdate implements Action {

			// Timer
			private ElapsedTime timer;

			// Duration in milliseconds
			private double milliseconds;

			// Initialized value
			private boolean initialized;

			// Initialzies this.
			public WaitAndUpdate(double milliseconds) {

				// Remember the duration in milliseconds.
				this.milliseconds = milliseconds;

			}

			// Runs this.
			@Override
			public boolean run(@NonNull TelemetryPacket packet) {

				// If this is not initialized...
				if (!initialized) {

					// Start a timer.
					timer = new ElapsedTime();

					// Remember that this is initialized.
					initialized = true;
				}

				// Update the robot hardware.
				robotHardware.update();

				// Determine whether we are waiting.
				boolean waiting = timer.milliseconds() < milliseconds;

				// Return the result.
				return waiting;

			}

		}

		// Waits for the robot hardware to finish moving.
		public class WaitForNotBusy implements Action {

			// Runs this.
			@Override
			public boolean run(@NonNull TelemetryPacket packet) {

				// Update the robot hardware.
				robotHardware.update();

				// Determine whether the robot hardware is busy.
				boolean isBusy = robotHardware.isArmBusy() || robotHardware.isLiftBusy() || robotHardware.isSlideBusy();

				// Return the result.
				return isBusy;

			}

		}

	}