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

    public static Pose2d START_POSE = new Pose2d(-38, -62, 0);
    public static Pose2d BASKET_POSE = new Pose2d(-50, -50, 1.0 / 4 * Math.PI);
    public static Pose2d FIRST_SPIKE_MARK_POSE = new Pose2d(-50, -38, 1.0 / 2 * Math.PI);
    public static int HIGH_BASKET_DELAY = 3000;
    public static int HIGH_BASKET_TO_GROUND_DELAY = 3000;
    public static int CLAW_DELAY = 2000;
    public static int HIGH_BASKET_TO_LOW_BASKET_DELAY = 2000;
    public static int LOW_BASKET_TO_ALMOST_GROUND_DELAY = 2000;
    public static int ALMOST_GROUND_TO_GROUND_DELAY = 1000;

    private RobotHardware robotHardware;

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

        // Construct a drive interface.
        MecanumDrive drive = new MecanumDrive(hardwareMap, START_POSE);

        // Construct an action for driving from the start to the basket.
        Action driveFromStartToBasket = drive.actionBuilder(START_POSE)
                .strafeToLinearHeading(BASKET_POSE.position, BASKET_POSE.heading)
                .build();

        // Construct an action for driving from the basket to the first spike mark.
        Action driveFromBasketToFirstSpikeMark = drive.actionBuilder(BASKET_POSE)
                .strafeToLinearHeading(FIRST_SPIKE_MARK_POSE.position, FIRST_SPIKE_MARK_POSE.heading)
                .build();

        // Construct an action for driving from the first spike mark to the basket.
        Action driveFromFirstSpikeMarkToBasket = drive.actionBuilder(FIRST_SPIKE_MARK_POSE)
                .strafeToLinearHeading(BASKET_POSE.position, BASKET_POSE.heading)
                .build();

        /*lastPose = new Pose2d(BASKET_DIAGONAL_POSITION, BASKET_DIAGONAL_POSITION, BASKET_DIAGONAL_HEADING);

        Action driveToBasket2 = drive.actionBuilder(lastPose)
                .strafeToLinearHeading(new Vector2d(BASKET_DIAGONAL_POSITION, BASKET_DIAGONAL_POSITION), BASKET_DIAGONAL_HEADING)
                .build();
        lastPose = new Pose2d(BASKET_DIAGONAL_POSITION, BASKET_DIAGONAL_POSITION, BASKET_DIAGONAL_HEADING);

        Action driveToSpikeMark2 = drive.actionBuilder(lastPose)
                .strafeToLinearHeading(new Vector2d(FIRST_NEUTRAL_SPIKE_MARK_X, FIRST_NEUTRAL_SPIKE_MARK_Y), FIRST_NEUTRAL_SPIKE_MARK_HEADING)
                .build();
        lastPose = new Pose2d(BASKET_DIAGONAL_POSITION, BASKET_DIAGONAL_POSITION, BASKET_DIAGONAL_HEADING);

        Action driveToBasket3 = drive.actionBuilder(lastPose)
                .strafeToLinearHeading(new Vector2d(BASKET_DIAGONAL_POSITION, BASKET_DIAGONAL_POSITION), BASKET_DIAGONAL_HEADING)
                .build();
        lastPose = new Pose2d(BASKET_DIAGONAL_POSITION, BASKET_DIAGONAL_POSITION, BASKET_DIAGONAL_HEADING);

        Action driveToSpikeMark3 = drive.actionBuilder(lastPose)
                .strafeToLinearHeading(new Vector2d(FIRST_NEUTRAL_SPIKE_MARK_X, FIRST_NEUTRAL_SPIKE_MARK_Y), FIRST_NEUTRAL_SPIKE_MARK_HEADING)
                .build();*/

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
                        new WaitForNotBusy()

                )
        );

        /*
        // Drive To Basket
        Pose2d lastPose = new Pose2d(-38, -62, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, lastPose);

        waitForStart();

        Actions.runBlocking(drive.actionBuilder(lastPose)
                .strafeToLinearHeading(new Vector2d(BASKET_DIAGONAL_POSITION, BASKET_DIAGONAL_POSITION), 1.0 / 4 * Math.PI)
                .build());

        // Reset the timer.
        timer.reset();

        while (timer.milliseconds() < 2000) {
            // Move the arm to the high basket position.
            robotHardware.setArmHighBasketAutoPosition();

            // Move the lift to the high basket position
            robotHardware.setLiftHighBasketPosition();

            // Use the high basket extension.
            robotHardware.setHighBasketExtension();

            // Update the robot hardware.
            robotHardware.update();
        }

        // Reset the timer.
        timer.reset();

        // Toggle the claw.
        robotHardware.toggleClaw();

        while (timer.milliseconds() < 2000) {
            // Update the robot hardware.
            robotHardware.update();
        }

        // Reset the timer.
        timer.reset();

        while (timer.milliseconds() < 2000) {
            // Move the lift to the ground position
            robotHardware.setLiftGroundPosition();

            // Use the low basket extension.
            robotHardware.setLowBasketExtension();

            // Update the robot hardware.
            robotHardware.update();
        }

        // Reset the timer.
        timer.reset();

        while (timer.milliseconds() < 2000) {
            // Move the arm to the ground position.
            robotHardware.setArmAlmostGroundPosition();

            // Fully retract the slide.
            robotHardware.setMinimumExtension();

            // Update the robot hardware.
            robotHardware.update();
        }

        // Reset the timer.
        timer.reset();

        while (timer.milliseconds() < 2000) {
            // Move the arm to the ground position.
            robotHardware.setArmGroundPosition();

            // Update the robot hardware.
            robotHardware.update();
        }

        lastPose = new Pose2d(BASKET_DIAGONAL_POSITION, BASKET_DIAGONAL_POSITION, 1.0 / 4 * Math.PI);
        Actions.runBlocking(drive.actionBuilder(lastPose)
                .strafeToLinearHeading(new Vector2d(FIRST_NEUTRAL_SPIKE_MARK_X, FIRST_NEUTRAL_SPIKE_MARK_Y), 1.0 / 2 * Math.PI)
                .build());

        // Reset the timer.
        timer.reset();

        // Toggle the claw.
        robotHardware.toggleClaw();

        while (timer.milliseconds() < 2000) {
            // Update the robot hardware.
            robotHardware.update();
        }

        lastPose = new Pose2d(FIRST_NEUTRAL_SPIKE_MARK_X, FIRST_NEUTRAL_SPIKE_MARK_Y, 1.0 / 2 * Math.PI);
        Actions.runBlocking(drive.actionBuilder(lastPose)
                .strafeToLinearHeading(new Vector2d(BASKET_DIAGONAL_POSITION, BASKET_DIAGONAL_POSITION), 1.0 / 4 * Math.PI)
                .build());

        // Reset the timer.
        timer.reset();

        while (timer.milliseconds() < 2000) {
            // Move the arm to the high basket position.
            robotHardware.setArmHighBasketAutoPosition();

            // Move the lift to the high basket position
            robotHardware.setLiftHighBasketPosition();

            // Use the high basket extension.
            robotHardware.setHighBasketExtension();

            // Update the robot hardware.
            robotHardware.update();
        }

        // Reset the timer.
        timer.reset();

        // Toggle the claw.
        robotHardware.toggleClaw();

        while (timer.milliseconds() < 2000) {
            // Update the robot hardware.
            robotHardware.update();
        }

        // Reset the timer.
        timer.reset();

        while (timer.milliseconds() < 2000) {
            // Move the arm to the low basket position.
            robotHardware.setArmLowBasketPosition();

            // Move the lift to the ground position
            robotHardware.setLiftGroundPosition();

            // Use the low basket extension.
            robotHardware.setLowBasketExtension();

            // Update the robot hardware.
            robotHardware.update();
        }

        // Reset the timer.
        timer.reset();

        while (timer.milliseconds() < 2000) {
            // Move the arm to the ground position.
            robotHardware.setArmAlmostGroundPosition();

            // Fully retract the slide.
            robotHardware.setMinimumExtension();

            // Update the robot hardware.
            robotHardware.update();
        }

        // Reset the timer.
        timer.reset();

        while (timer.milliseconds() < 2000) {
            // Move the arm to the ground position.
            robotHardware.setArmGroundPosition();

            // Update the robot hardware.
            robotHardware.update();
        }

        // Update the robot hardware.
        robotHardware.update();

        // Update the telemetry.
        telemetry.update();
         */

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

    /*
    public class Default implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            return false;
        }
    }
     */
}