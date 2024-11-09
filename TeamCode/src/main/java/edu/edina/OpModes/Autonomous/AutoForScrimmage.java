package edu.edina.OpModes.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
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


    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // Get hardware.
        RobotHardware robotHardware = new RobotHardware(this);

        // Wait for the user to lower the lift.
        robotHardware.waitForLiftDown();

        // Wait for the user to lower the arm.
        robotHardware.waitForArmDown();

        // Initialize the robot.
        robotHardware.initializeRobot();

        // If stop is requested...
        if(isStopRequested()) {

            // Exit the method.
            return;

        }

        // Prompt the user to press start.
        robotHardware.log("Waiting for start...");

        // Wait for the user to press start.
        waitForStart();

        // Drive To Basket
        Pose2d beginPose = new Pose2d(-38, -62, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(drive.actionBuilder(beginPose)
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

        beginPose = new Pose2d(BASKET_DIAGONAL_POSITION, BASKET_DIAGONAL_POSITION, 1.0 / 4 * Math.PI);
        Actions.runBlocking(drive.actionBuilder(beginPose)
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

        beginPose = new Pose2d(FIRST_NEUTRAL_SPIKE_MARK_X, FIRST_NEUTRAL_SPIKE_MARK_Y, 1.0 / 2 * Math.PI);
        Actions.runBlocking(drive.actionBuilder(beginPose)
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

    }
}