package edu.edina.OpModes.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.Robot.RobotHardware;

@Config
@Autonomous(preselectTeleOp = "TeleOpForScrimmage")
public class AutoForScrimmage extends LinearOpMode {
    public static double BASKET_DIAGONAL_POSITION = -50;
    public static double FIRST_NEUTRAL_SPIKE_MARK_X = -50;
    public static double FIRST_NEUTRAL_SPIKE_MARK_Y = -38;
    public static int HIGH_BASKET_DELAY = 3000;
    public static int CLAW_DELAY = 2000;
    public static int HIGH_BASKET_TO_LOW_BASKET_DELAY = 2000;
    public static int LOW_BASKET_TO_ALMOST_GROUND_DELAY = 2000;
    public static int ALMOST_GROUND_TO_GROUND_DELAY = 1000;


    private RobotHardware robotHardware;

    private Pose2d lastEnd;

    @Override
    public void runOpMode() throws InterruptedException {

        // Get hardware.
        robotHardware = new RobotHardware(this);

        // Wait for the user to lower the lift.
        robotHardware.waitForLiftDown();

        // Wait for the user to lower the arm.
        robotHardware.waitForArmDown();

        // Initialize the robot.
        robotHardware.initializeRobot();

        // If stop is requested...
        if (isStopRequested()) {

            // Exit the method.
            return;

        }

        // Prompt the user to press start.
        robotHardware.log("Waiting for start...");

        // Wait for the user to press start.
        waitForStart();

        Pose2d lastPose = new Pose2d(-38, -62, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, lastPose);

        Action driveToBasket0 = drive.actionBuilder(lastPose)
                .strafeToLinearHeading(new Vector2d(BASKET_DIAGONAL_POSITION, BASKET_DIAGONAL_POSITION), 1.0 / 4 * Math.PI)
                .build();
        lastPose = new Pose2d(BASKET_DIAGONAL_POSITION, BASKET_DIAGONAL_POSITION, 1.0 / 4 * Math.PI);

        Action driveToSpikeMark1 = drive.actionBuilder(lastPose)
                .strafeToLinearHeading(new Vector2d(FIRST_NEUTRAL_SPIKE_MARK_X, FIRST_NEUTRAL_SPIKE_MARK_Y), 1.0 / 2 * Math.PI)
                .build();
        lastPose = new Pose2d(FIRST_NEUTRAL_SPIKE_MARK_X, FIRST_NEUTRAL_SPIKE_MARK_Y, 1.0 / 2 * Math.PI);

        Action driveToBasket1 = drive.actionBuilder(lastPose)
                .strafeToLinearHeading(new Vector2d(BASKET_DIAGONAL_POSITION, BASKET_DIAGONAL_POSITION), 1.0 / 4 * Math.PI)
                .build();
        /*lastPose = new Pose2d(BASKET_DIAGONAL_POSITION, BASKET_DIAGONAL_POSITION, 1.0 / 4 * Math.PI);

        Action driveToBasket2 = drive.actionBuilder(lastPose)
                .strafeToLinearHeading(new Vector2d(BASKET_DIAGONAL_POSITION, BASKET_DIAGONAL_POSITION), 1.0 / 4 * Math.PI)
                .build();
        lastPose = new Pose2d(BASKET_DIAGONAL_POSITION, BASKET_DIAGONAL_POSITION, 1.0 / 4 * Math.PI);

        Action driveToSpikeMark2 = drive.actionBuilder(lastPose)
                .strafeToLinearHeading(new Vector2d(FIRST_NEUTRAL_SPIKE_MARK_X, FIRST_NEUTRAL_SPIKE_MARK_Y), 1.0 / 2 * Math.PI)
                .build();
        lastPose = new Pose2d(BASKET_DIAGONAL_POSITION, BASKET_DIAGONAL_POSITION, 1.0 / 4 * Math.PI);

        Action driveToBasket3 = drive.actionBuilder(lastPose)
                .strafeToLinearHeading(new Vector2d(BASKET_DIAGONAL_POSITION, BASKET_DIAGONAL_POSITION), 1.0 / 4 * Math.PI)
                .build();
        lastPose = new Pose2d(BASKET_DIAGONAL_POSITION, BASKET_DIAGONAL_POSITION, 1.0 / 4 * Math.PI);

        Action driveToSpikeMark3 = drive.actionBuilder(lastPose)
                .strafeToLinearHeading(new Vector2d(FIRST_NEUTRAL_SPIKE_MARK_X, FIRST_NEUTRAL_SPIKE_MARK_Y), 1.0 / 2 * Math.PI)
                .build();*/

        Actions.runBlocking(
                new SequentialAction(
                        // Declared above, blocks while running
                        driveToBasket0,
                        // Sets positions, runs once
                        new MoveToHighBasket(),
                        // Waits and updates robot hardware
                        new WaitAndUpdate(HIGH_BASKET_DELAY),
                        // Opens the claw, runs once
                        new OpenClaw(),
                        new WaitAndUpdate(CLAW_DELAY),
                        new MoveToLowBasket(),

                        // Waiting
                        new WaitForLiftNotBusy(),
                        new WaitForArmNotBusy(),
                        // Wait for extension delay (and wrist?)

                        new MoveToAlmostGround(),

                        // Waiting
                        new WaitForLiftNotBusy(),
                        new WaitForArmNotBusy(),
                        // Wait for extension delay (and wrist?)

                        new MoveToGround(),

                        // Waiting
                        new WaitForLiftNotBusy(),
                        new WaitForArmNotBusy(),
                        // Wait for extension delay (and wrist?)

                        driveToSpikeMark1,
                        new CloseClaw(),
                        new WaitAndUpdate(CLAW_DELAY),
                        driveToBasket1,
                        new MoveToHighBasket(),

                        // Waiting
                        new WaitForLiftNotBusy(),
                        new WaitForArmNotBusy(),
                        // Wait for extension delay (and wrist?)

                        new OpenClaw(),
                        new WaitAndUpdate(CLAW_DELAY),
                        new MoveToLowBasket(),

                        // Waiting
                        new WaitForLiftNotBusy(),
                        new WaitForArmNotBusy(),
                        // Wait for extension delay (and wrist?)

                        new MoveToAlmostGround(),

                        // Waiting
                        new WaitForLiftNotBusy(),
                        new WaitForArmNotBusy(),
                        // Wait for extension delay (and wrist?)

                        new MoveToGround(),

                        // Waiting
                        new WaitForLiftNotBusy(),
                        new WaitForArmNotBusy()
                        // Wait for extension delay (and wrist?)

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

    public class OpenClaw implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robotHardware.openClaw();
            return false;
        }
    }

    public class CloseClaw implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robotHardware.closeClaw();
            return false;
        }
    }

    public class MoveToHighBasket implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // Move the arm to the high basket position.
            robotHardware.setArmHighBasketAutoPosition();

            // Move the lift to the high basket position
            robotHardware.setLiftHighBasketPosition();

            // Use the high basket extension.
            robotHardware.setHighBasketExtension();
            return false;
        }
    }

    public class MoveToLowBasket implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // Move the arm to the low basket position.
            robotHardware.setArmLowBasketPosition();

            // Move the lift to the ground position
            robotHardware.setLiftGroundPosition();

            // Use the low basket extension.
            robotHardware.setLowBasketExtension();
            return false;
        }
    }

    public class MoveToAlmostGround implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // Move the arm to the ground position.
            robotHardware.setArmAlmostGroundPosition();

            // Move the lift to the ground position
            robotHardware.setLiftGroundPosition();

            // Fully retract the slide.
            robotHardware.setMinimumExtension();
            return false;
        }
    }

    public class MoveToGround implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // Move the arm to the ground position.
            robotHardware.setArmGroundPosition();

            // Move the lift to the ground position
            robotHardware.setLiftGroundPosition();

            // Fully retract the slide.
            robotHardware.setMinimumExtension();
            return false;
        }
    }

    public class WaitAndUpdate implements Action {

        private ElapsedTime timer;
        private double milliseconds;
        private boolean initialized;

        public WaitAndUpdate(double milliseconds) {
            this.milliseconds = milliseconds;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer = new ElapsedTime();
                initialized = true;
            }

            robotHardware.update();

            return timer.milliseconds() < milliseconds;
        }
    }

    public class WaitForLiftNotBusy implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robotHardware.update();
            return !robotHardware.getIsLiftBusy();
        }
    }

    public class WaitForArmNotBusy implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robotHardware.update();
            return !robotHardware.getIsArmBusy();
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