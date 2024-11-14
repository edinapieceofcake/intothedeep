package edu.edina.OpModes.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.Robot.RobotHardware;

@Config
@Autonomous(preselectTeleOp = "TeleOpForScrimmage")
public class AutoForScrimmage extends LinearOpMode {

    // Start pose
    public static double START_X = -38;
    public static double START_Y = -62;
    public static double START_HEADING = 0;

    // Basket pose
    public static double BASKET_X = -50;
    public static double BASKET_Y = -50;
    public static double BASKET_HEADING = 1.0 / 4 * Math.PI;

    // First spike mark pose
    public static double FIRST_SPIKE_MARK_X = -47.5;
    public static double FIRST_SPIKE_MARK_Y = -38;
    public static double FIRST_SPIKE_MARK_HEADING = 1.0 / 2 * Math.PI;

    // Second spike mark pose
    public static double SECOND_SPIKE_MARK_X = -57.5;
    public static double SECOND_SPIKE_MARK_Y = FIRST_SPIKE_MARK_Y;
    public static double SECOND_SPIKE_MARK_HEADING = FIRST_SPIKE_MARK_HEADING;

    // Duration in milliseconds to toggle the claw
    public static int CLAW_DELAY = 2000;

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

        // Construct a basket pose.
        Pose2d basketPose = new Pose2d(BASKET_X, BASKET_Y, BASKET_HEADING);

        // Construct a first spike mark pose.
        Pose2d firstSpikeMarkPose = new Pose2d(FIRST_SPIKE_MARK_X, FIRST_SPIKE_MARK_Y, FIRST_SPIKE_MARK_HEADING);

        // Construct a second spike mark pose.
        Pose2d secondSpikeMarkPose = new Pose2d(SECOND_SPIKE_MARK_X, SECOND_SPIKE_MARK_Y, SECOND_SPIKE_MARK_HEADING);

        // Construct a drive interface.
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Construct an action for driving from the start to the basket.
        Action driveFromStartToBasket = drive.actionBuilder(startPose)
                .strafeToLinearHeading(basketPose.position, basketPose.heading)
                .build();

        // Construct an action for driving from the basket to the first spike mark.
        Action driveFromBasketToFirstSpikeMark = drive.actionBuilder(basketPose)
                .strafeToLinearHeading(firstSpikeMarkPose.position, firstSpikeMarkPose.heading)
                .build();

        // Construct an action for driving from the first spike mark to the basket.
        Action driveFromFirstSpikeMarkToBasket = drive.actionBuilder(firstSpikeMarkPose)
                .strafeToLinearHeading(basketPose.position, basketPose.heading)
                .build();

        // Construct an action for driving from the basket to the second spike mark.
        Action driveFromBasketToSecondSpikeMark = drive.actionBuilder(basketPose)
                .strafeToLinearHeading(secondSpikeMarkPose.position, secondSpikeMarkPose.heading)
                .build();

        // Construct an action for driving from the second spike mark to the basket.
        Action driveFromSecondSpikeMarkToBasket = drive.actionBuilder(secondSpikeMarkPose)
                .strafeToLinearHeading(basketPose.position, basketPose.heading)
                .build();

        // Run the actions.
        Actions.runBlocking(
                new SequentialAction(

                        // Score preloaded sample
                        driveFromStartToBasket,
                        new MoveToHighBasket(),
                        new WaitForNotBusy(),
                        new OpenClaw(),
                        new WaitAndUpdate(CLAW_DELAY),
                        new MoveToGround(),
                        new WaitForNotBusy(),

                        // Score first spike mark sample
                        driveFromBasketToFirstSpikeMark,
                        new CloseClaw(),
                        new WaitAndUpdate(CLAW_DELAY),
                        driveFromFirstSpikeMarkToBasket,
                        new MoveToHighBasket(),
                        new WaitForNotBusy(),
                        new OpenClaw(),
                        new WaitAndUpdate(CLAW_DELAY),
                        new MoveToGround(),
                        new WaitForNotBusy(),

                        // Score second spike mark sample
                        driveFromBasketToSecondSpikeMark,
                        new CloseClaw(),
                        new WaitAndUpdate(CLAW_DELAY),
                        driveFromSecondSpikeMarkToBasket,
                        new MoveToHighBasket(),
                        new WaitForNotBusy(),
                        new OpenClaw(),
                        new WaitAndUpdate(CLAW_DELAY),
                        new MoveToGround(),
                        new WaitForNotBusy()

                )

        );

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

    // Moves the claw to the high basket.
    public class MoveToHighBasket implements Action {

        // Runs this.
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            // Move the arm to the high basket position.
            robotHardware.setArmHighBasketAutoPosition();

            // Move the lift to the high basket position
            robotHardware.setLiftHighBasketPosition();

            // Use the high basket extension.
            robotHardware.setHighBasketExtension();

            // Lower the wrist.
            robotHardware.lowerWrist();

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

    // Waits for a specified duration.
    public class WaitAndUpdate implements Action {

        // Timer
        private ElapsedTime timer;

        // Duration in milliseconds
        private double milliseconds;

        // Initialized value
        private boolean initialized;

        // Initialzies this.
        public WaitAndUpdate(double milliseconds)
        {

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