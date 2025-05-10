package edu.edina.Tests;

import static edu.edina.Libraries.Robot.RobotHardware.BLUE_SQUARE;
import static edu.edina.Libraries.Robot.RobotHardware.getBanner;
import static edu.edina.Libraries.Robot.RobotHardware.prompt;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.Robot.Arm;
import edu.edina.Libraries.Robot.MoveArm;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.WaitForTime;

@Config
@Autonomous(preselectTeleOp = "TeleOpMain")
public class RRTest extends LinearOpMode {
    // Start pose
    public static double START_X = 3;
    public static double START_Y = -60;
    public static double START_HEADING = Math.toRadians(270);

    // Preload chamber pose
    public static double PRELOAD_CHAMBER_X = START_X;
    public static double PRELOAD_CHAMBER_Y = -29;
    public static double PRELOAD_CHAMBER_HEADING = START_HEADING;
    ;

    // Robot hardware
    private RobotHardware robotHardware;

    // Runs the op mode.
    @Override
    public void runOpMode() throws InterruptedException {

        // Display launch menu.
        //////////////////////////////////////////////////////////////////////

        // Initialize an input tall walls value.
        Boolean inputTallWalls = null;

        // Initialize gamepads.
        Gamepad currentGamepad = new Gamepad();
        Gamepad previousGamepad = new Gamepad();

        // While we are waiting for a response...
        while (!isStopRequested()) {

            // Update gamepads.
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            // If the tall walls value is missing...
            if(inputTallWalls == null) {

                // Prompt the user for a walls value.
                prompt(telemetry, "Walls", "X = Tall, B = Short");

                // If the user pressed x...
                if (currentGamepad.x && !previousGamepad.x) {

                    // Use tall walls.
                    inputTallWalls = true;

                }

                // Otherwise, if the user pressed b...
                else if (currentGamepad.b && !previousGamepad.b) {

                    // Use short walls.
                    inputTallWalls = false;

                }

            }

            // Otherwise (if we are done)...
            else {

                // Hide the launch menu.
                break;

            }

        }

        // If stop is requested...
        if (isStopRequested()) {

            // Exit the method.
            return;

        }

        // Initialize the robot.
        //////////////////////////////////////////////////////////////////////

        // Construct a start pose.
        Pose2d startPose = new Pose2d(START_X, START_Y, START_HEADING);

        // Get hardware.
        robotHardware = new RobotHardware(this, startPose);

        // Indicate that the robot is initializing.
        robotHardware.log("Initializing...");

        // Set the tall walls value.
        robotHardware.setTallWalls(inputTallWalls);

        // Wait for the user to lower the lift.
        robotHardware.waitForLiftDown();

        // Wait for the user to lower the arm.
        robotHardware.waitForArmDown();
        // If stop is requested...
        if (isStopRequested()) {

            // Exit the method.
            return;

        }

        // Wait for start.
        //////////////////////////////////////////////////////////////////////

        // Add telemetry.
        addTelemetry();

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

        // Get a drive interface.
        MecanumDrive drive = robotHardware.getDrive();

        // Get the main action.
        Action mainAction = getMainAction(drive, startPose);

        // Add the main action to the robot hardware.
        robotHardware.addAction(mainAction);

        // While the op mode is active...
        while (opModeIsActive()) {

            // Add telemetry.
            addTelemetry();

            // Update the robot hardware.
            robotHardware.update();

            // Update the telemetry.
            telemetry.update();

        }

    }

    // Gets the main action.
    private Action getMainAction(MecanumDrive drive, Pose2d startPose) {
        // Construct a preload velocity constraint.
        // Construct a chamber pose.
        Pose2d chamberPose = new Pose2d(PRELOAD_CHAMBER_X, PRELOAD_CHAMBER_Y, PRELOAD_CHAMBER_HEADING);

        //////////////////////////////////////////////////////////////////////

        // Construct a drive from start to chamber trajectory.
        TrajectoryActionBuilder driveFromStartToChamberBuilder = drive.actionBuilder(startPose)
                .strafeTo(chamberPose.position);
        Action driveFromStartToChamber = driveFromStartToChamberBuilder.build();


        // Construct a drive nowhere trajectory.
        Action driveNowhere = new SequentialAction();

        // Construct a main action.
        //////////////////////////////////////////////////////////////////////

        // Get the tall walls value.
        boolean tallWalls = robotHardware.getTallWalls();

        // Construct a main action.
        Action mainAction = new SequentialAction(
                driveFromStartToChamber
        );

        // Return the main action.
        return mainAction;

    }
    // Adds telemetry.
    private void addTelemetry() {

        // Get a banner.
        String banner = getBanner(BLUE_SQUARE);

        // Get the tall walls value.
        boolean tallWalls = robotHardware.getTallWalls();

        // Construct a walls string.
        String walls = tallWalls ? "Tall" : "Short";

        // Display main telemetry.
        telemetry.addData("AutoSpecimen", banner);
        telemetry.addData("- Walls", walls);

    }
}
