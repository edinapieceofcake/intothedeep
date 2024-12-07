package edu.edina.Libraries.Robot;

import static edu.edina.OpModes.Autonomous.AutoSample.TIMEOUT_MILLISECONDS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.RoadRunner.ThreeDeadWheelLocalizer;
import edu.edina.OpModes.Autonomous.AutoSample;
import edu.edina.OpModes.Autonomous.AutoSpecimen;

@Config
public class RobotHardware {
    /*
    Control Hub Portal
        Control Hub
            Motors
                0 - GoBILDA 5201 series - right_lift_motor (has right lift encoder)
                1 - GoBILDA 5201 series - front_encoder (is front odometry encoder)
                2 - GoBILDA 5201 series - left_back_drive (encoder port returns 0 and -1)
                3 - GoBILDA 5201 series - left_front_drive (has left odometry encoder)
            Servos
                0 - GoBILDA torque servo - wrist_left
                1 - GoBILDA speed servo - claw_servo
                2 - Axon MINI+ servo - swivel_servo
            Digital Devices
                5 - REV Touch Sensor - arm_touch
                7 - REV Touch Sensor - lift_touch
            I2C
                3 - Neopixel Driver - neopixel_driver
            Analog
                0 - Axon Micro+ ServoE - wrist_left_encoder
                1 - Axon Micro+ ServoF - wrist_right_encoder
        Expansion Hub 2
            Motors
                0 - GoBILDA 5201 series - left_lift_motor (has left lift encoder)
                1 - GoBILDA 5201 series - arm_motor (has through bore encoder)
                2 - GoBILDA 5201 series - right_front_drive (encoder port has bent pin)
                3 - GoBILDA 5201 series - right_back_drive (has right odometry encoder)
            Servos
                2 - CRServo Axon Mini+ - slide_servo
            Analog
                0 - Axon Mini+ Encoder - slide_encoder
    */

    // Inches to back up when scoring a specimen
    public static int SCORE_SPECIMEN_BACKUP_INCHES = 7;

    private final LinearOpMode opMode;
    public final IMU imu;
    public ThreeDeadWheelLocalizer odometry;
    public final MecanumDrive drive;
    public final VoltageSensor voltageSensor;
    private Drivetrain drivetrain;
    private final Wrist wrist;
    private final Swivel swivel;
    private final Arm arm;
    private final Claw claw;
    private final Lift lift;
    private final Slide slide;
    private final BoundingBoxFailsafe failsafe;
    //private Light light;
    //private SampleSensor sampleSensor;
    private boolean turtleMode;
    private int beepSoundId;
    private List<Action> runningActions = new ArrayList<>();
    private FtcDashboard dashboard;
    private boolean allowManualDriving = true;

    public RobotHardware(LinearOpMode opMode) throws InterruptedException {

        // Remember the op mode.
        this.opMode = opMode;

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        //sampleSensor = new SampleSensor(hardwareMap);

        // Get an FTC dashboard instance.
        dashboard = FtcDashboard.getInstance();

        // Get the beep sound identifier.
        beepSoundId = hardwareMap.appContext.getResources().getIdentifier("beep", "raw", hardwareMap.appContext.getPackageName());

        // If the beep sound is missing...
        if (beepSoundId == 0) {

            // Complain.
            throw new InterruptedException("The beep sound file is missing.");

        }

        // Preload the beep sound.
        SoundPlayer.getInstance().preload(hardwareMap.appContext, beepSoundId);

        // Initialize the arm.
        arm = new Arm(this);

        // Initialize the claw.
        claw = new Claw(this);

        // Initialize the lift.
        lift = new Lift(this);

        // Initialize the slide.
        slide = new Slide(this);

        // Initialize the wrist.
        wrist = new Wrist(hardwareMap, opMode.telemetry);

        // Initialize the swivel.
        swivel = new Swivel(hardwareMap, opMode.telemetry);

        // Initialize the drivetrain.
        drivetrain = new Drivetrain(opMode);

        failsafe = new BoundingBoxFailsafe(wrist, arm, lift, slide);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        odometry = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        //light = new Light(hardwareMap, sampleSensor);
    }

    // Waits for the user to lower the lift.
    public void waitForLiftDown() {

        // Waits for the user to lower the lift.
        lift.waitForDown();

    }

    // Waits for the user to lower the arm.
    public void waitForArmDown() throws InterruptedException {

        // Waits for the user to lower the arm.
        arm.waitForDown();

    }

    // Logs a message.
    public void log(String message) {

        // If the op mode is missing...
        if (opMode == null) {

            // Exit the method.
            return;

        }

        // Get the telemetry.
        Telemetry telemetry = opMode.telemetry;

        // Show the message.
        telemetry.addData("Message", message);
        telemetry.update();

    }

    // Toggles the wrist.
    public void toggleWrist() {

        // Toggle the wrist.
        wrist.toggle();

    }

    // Raises the wrist.
    public void raiseWrist() {

        // Raise the wrist.
        wrist.raise();

    }

    // Initializes the wrist.
    public void initializeWrist() {

        // Raise the wrist.
        wrist.initialize();

    }

    // Lowers the wrist.
    public void lowerWrist() {

        // Lowers the wrist.
        wrist.lower();

    }

    // Sets the swivel to horizontal.
    public void swivelSetHorizontal() {

        // Sets the swivel to horizontal.
        swivel.setHorizontal();

    }

    // Sets the swivel to vertical.
    public void swivelSetVertical() {

        // Sets the swivel to horizontal.
        swivel.setHorizontal();

    }

    // Sets the swivel to clip.
    public void swivelSetClip() {

        // Sets the swivel to clip.
        swivel.setClip();

    }

    // Toggles the swivel.
    public void toggleSwivel() {

        // Toggles the swivel.
        swivel.toggle();

    }

    public void updateHardwareInteractions() {

        // RM: I commented out the following code because it was causing unexpected behavior during
        // tele op I do not think toggling the wrist is necessary to keep the robot within the
        // expansion box.

        /*
        if (arm.armWillCrossWristLimit())
            raiseWrist();
        else if (arm.targetingScoringPos() || arm.isInHighBar() || arm.isInLowBar())
            lowerWrist();
        */

        drivetrain.setAutoTurtleMode(lift.isRaised());

    }

    public void setReversed(boolean reversed) {
        drivetrain.setReverse(reversed);
    }

    // Updates this.
    public void update() {

        // Update hardware.
        //////////////////////////////////////////////////////////////////////

        // Set turtle mode.
        drivetrain.setTurtleMode(turtleMode);

        // Update the arm.
        arm.update();

        // Update the claw.
        claw.update();

        // If manual driving is allowed...
        if(allowManualDriving) {

            // Update the drivetrain.
            drivetrain.update();

        }

        // Update the lift.
        lift.update();

        // Update the slide.
        slide.update();

        // Update the wrist.
        wrist.update();

        // Update the swivel.
        swivel.update();

        // Update the light.
        //light.update();

        // Update actions.
        //////////////////////////////////////////////////////////////////////

        // Road Runner Docs - Teleop Actions
        // https://rr.brott.dev/docs/v1-0/guides/teleop-actions/

        // Construct a telemetry packet.
        TelemetryPacket packet = new TelemetryPacket();

        // Construct a new action list.
        List<Action> newActions = new ArrayList<>();

        // For each running action...
        for (Action action : runningActions) {

            // Update the dashboard.
            action.preview(packet.fieldOverlay());

            // If the action is still running...
            if (action.run(packet)) {

                // Add the action to the new action list.
                newActions.add(action);

            }

        }

        // Update the running actions.
        runningActions = newActions;

        // Update the dashboard.
        dashboard.sendTelemetryPacket(packet);

    }

    // Decrements the arm position.
    public void decrementArmPosition() {

        // Decrement the arm position.
        arm.decrementPosition();

    }

    // Increments the arm position.
    public void incrementArmPosition() {

        // Increment the arm position.
        arm.incrementPosition();

    }

    // Toggles the updateHardwareInteractions.
    public void toggleClaw() {

        // Toggle the claw.
        claw.toggle();

    }

    // Opens the claw.
    public void openClaw() {

        // Open the claw.
        claw.open();

    }

    // Closes the claw.
    public void closeClaw() {

        // Close the claw.
        claw.close();

    }

    // Raises the lift.
    public void raiseLift() {

        // Raise the lift.
        lift.raise();

    }

    // Lowers the lift.
    public void lowerLift() {

        // Lower the lift.
        lift.lower();

    }

    // Gets the op mode.
    public LinearOpMode getOpMode() {

        // Return the op mode.
        return opMode;

    }

    // Sets the slide's position.
    public void setSlidePosition(double position) {

        // Set the slide's position.
        slide.setPosition(position);

    }

    // Retracts the slide.
    public void retractSlide() {

        // Retract the slide.
        slide.retract();

    }

    // Extends the slide.
    public void extendSlide() {

        // Extend the slide.
        slide.extend();

    }

    // Sets the minimum extension.
    public void setMinimumExtension() {

        // Set the minimum extension.
        slide.setMinimumExtension();

    }

    // Sets the low basket extension.
    public void setLowBasketExtension() {

        // Set the low basket extension.
        slide.setLowBasketExtension();

    }

    // Sets the high basket extension.
    public void setHighBasketExtension() {

        // Set the high basket extension.
        slide.setHighBasketExtension();

    }

    // Toggles turtle mode.
    public void toggleTurtleMode() {

        // Toggle turtle mode.
        turtleMode = !turtleMode;

    }

    // Sets turtle mode.
    public void setTurtleMode(boolean turtleMode) {

        // Set turtle mode.
        this.turtleMode = turtleMode;

    }
//
//    public boolean wristCanBeExtended() {
//        return arm.isForward();
//    }

    // Determines whether the arm is nearly down.
    public boolean isArmNearlyDown() {

        // Determine whether the arm is nearly down.
        boolean isNearlyDown = arm.isNearlyDown();

        // Return the result.
        return isNearlyDown;

    }

    // Determines whether the arm is near the submersible position.
    public boolean isArmNearSubmersiblePosition() {

        // Determine whether the arm is near the submersible position.
        boolean isArmNearSubmersiblePosition = arm.isNearSubmersiblePosition();

        // Return the result.
        return isArmNearSubmersiblePosition;

    }

    // Moves the arm to the ground position.
    public void setArmGroundPosition() {

        // Determine whether to move fast.
        boolean fast = arm.isNearSubmersiblePosition();

        // Construct an action to move the arm to the ground position.
        Action action = new SequentialAction(
                new MoveArm(this, Arm.GROUND_POSITION, fast),
                new WaitForHardware(this, TIMEOUT_MILLISECONDS),
                new InstantAction(() -> lowerWrist())
        );

        // Run the action.
        runningActions.add(action);

    }

    // Moves the arm to the low basket position.
    public void setArmLowBasketPosition() {

        // Construct an action to move the arm to the low basket position.
        Action action = new MoveArm(this, Arm.LOW_BASKET_POSITION, false);

        // Run the action.
        runningActions.add(action);

    }

    // Moves the arm to the ascent position.
    public void setArmAscentPosition() {

        // Moves the arm to the ascent position.
        arm.setAscentPosition();

    }

    // Moves the arm to the high basket position.
    public void setArmHighBasketPosition() {

        // Construct an action to move the arm to the high basket position.
        Action action = new MoveArm(this, Arm.HIGH_BASKET_POSITION, false);

        // Run the action.
        runningActions.add(action);

    }

    // Moves the arm to the high chamber position.
    public void setArmHighChamberPosition() {

        // Construct an action to move the arm to the high chamber position.
        Action action = new MoveArm(this, Arm.HIGH_CHAMBER_POSITION, true);

        // Run the action.
        runningActions.add(action);

    }

    // Moves the arm to the low chamber position.
    public void setArmLowChamberPosition() {

        // Construct an action to move the arm to the low chamber position.
        Action action = new MoveArm(this, Arm.LOW_CHAMBER_POSITION, true);

        // Run the action.
        runningActions.add(action);

    }

    // Moves the arm to the submersible position.
    public void setArmSubmersiblePosition() {

        // Construct an action to move the arm to the submersible position.
        Action action = new MoveArm(this, Arm.SUBMERSIBLE_POSITION, true);

        // Run the action.
        runningActions.add(action);

    }

    // Moves the lift to the ground position.
    public void setLiftGroundPosition() {

        // Move the lift to the ground position.
        lift.setGroundPosition();

    }

    // Moves the lift to the high basket position.
    public void setLiftHighBasketPosition() {

        // Move the lift to the high basket position.
        lift.setHighBasketPosition();

    }

    // Checks if the lift is busy.
    public boolean isLiftBusy() {

        // Checks if the lift is busy.
        return lift.isBusy();

    }

    // Checks if the arm is busy.
    public boolean isArmBusy() {

        // Checks if the arm is busy.
        return arm.isBusy();

    }

    // Checks if the slide is busy.
    public boolean isSlideBusy() {

        // Checks if the slide is busy.
        return slide.isBusy();

    }

    // Determines whether the arm is in the submersible position.
    public boolean isArmInSubmersiblePosition() {

        // Return indicating if the arm is in the submersible position.
        return arm.isInSubmersiblePosition();

    }

    // Determines whether the arm is in the low basket position.
    public boolean isArmInLowBasketPosition() {

        // Return indicating if the arm is in the low basket position.
        return arm.isInLowBasketPosition();

    }

    // Determines whether the arm is in the high basket position.
    public boolean isArmInHighBasketPosition() {

        // Return indicating if the arm is in the high basket position.
        return arm.isInHighBasketPosition();

    }

    // Determines whether the arm is in the ground position.
    public boolean isArmInGroundPosition() {

        // Return indicating if the arm is in the ground position.
        return arm.isInGroundPosition();

    }

    // Determines whether the arm is in the high chamber position.
    public boolean isArmInHighChamberPosition() {

        // Return indicating if the arm is in the high chamber position.
        return arm.isInHighChamberPosition();

    }

    // Gets the current slide extension.
    public double getCurrentSlideExtension() {

        // Return the current slide extension.
        return slide.getCurrentExtension();

    }

    public void setWristHighChamberHoldPosition() {
        wrist.setHighChamberHoldPosition();
    }
    public void setWristHighChamberClipPosition() {
        wrist.setHighChamberClipPosition();
    }

    public BoundingBoxFailsafe getFailsafe() {
        return failsafe;
    }

    // Beeps
    public void beep() {

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Play the beep sound.
        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, beepSoundId);

    }

    // Scores a specimen.
    public void scoreSpecimen() {

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Construct a start pose.
        Pose2d startPose = new Pose2d(0, 0, 0);

        // Construct an end pose.
        Pose2d endPose = new Pose2d(SCORE_SPECIMEN_BACKUP_INCHES, 0, 0);

        // Construct a drive interface.
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Construct a backup action.
        Action backup = drive.actionBuilder(startPose)
                .strafeToLinearHeading(endPose.position, endPose.heading)
                .build();

        // Construct a score specimen action.
        Action action = new SequentialAction(
                new InstantAction(() -> disableManualDriving()),
                AutoSpecimen.scoreSpecimen(backup,this),
                new InstantAction(() -> enableManualDriving())
        );

        // Run the action.
        runningActions.add(action);

    }

    // Scores a sample.
    public void scoreSample() {

        // Construct a score sample action.
        Action action = AutoSample.scoreSample(this);

        // Run the action.
        runningActions.add(action);

    }

    // Raises a sample.
    public void raiseSample() {

        // Construct a raise sample action.
        Action action = AutoSample.raiseSampleTeleOp(this);

        // Run the action.
        runningActions.add(action);

    }

    // Determines whether the lift is in the high basket position.
    public boolean isLiftInHighBasketPosition() {

        // Return indicating if the lift is in the high basket position.
        return lift.isInHighBasketPosition();

    }

    // Determines whether the lift is in the ground position.
    public boolean isLiftInGroundPosition() {

        // Return indicating if the lift is in the ground position.
        return lift.isInGroundPosition();

    }

    // Clears any pending actions.
    public void clearActions() {

        // Clear any pending actions.
        runningActions.clear();

        // Enable manual driving.
        enableManualDriving();

    }

    // Gets the arm's current position.
    public int getCurrentArmPosition() {

        // Return the arm's current position.
        return arm.getCurrentPosition();

    }

    // Sets the arm's position.
    public void setArmPosition(int position) {

        // Set the arm's position.
        arm.setPosition(position);

    }

    // Determines whether the slide is fully retracted.
    public boolean isSlideFullyRetracted() {

        // Return indicating if the slide is fully retracted.
        return slide.isFullyRetracted();

    }

    // Determines whether the slide is fully extended.
    public boolean isSlideFullyExtended() {

        // Return indicating if the slide is fully extended.
        return slide.isFullyExtended();

    }

    // Gets the lift's current position.
    public int getCurrentLiftPosition() {

        // Return the lift's current position.
        return (int)lift.getPosition();

    }

    // Sets the lift's position.
    public void setLiftPosition(int position) {

        // Set the lift's position.
        lift.setPosition(position);

    }

    // Ascends the robot.
    public void ascend() {

        // Construct an ascend action.
        Action action = new SequentialAction(
                new InstantAction(() -> setArmAscentPosition()),
                new Ascend(this)
        );

        // Run the action.
        runningActions.add(action);

    }

    // Descends the robot.
    public void descend() {

        // Construct a descend action.
        Action action = new SequentialAction(
                new InstantAction(() -> setArmAscentPosition()),
                new Descend(this)
        );

        // Run the action.
        runningActions.add(action);

    }

    // Starts arm rezeroing.
    public void startArmRezeroing() {

        // Start arm rezeroing.
        arm.startRezeroing();

    }

    // Stops arm rezeroing.
    public void stopArmRezeroing() {

        // Stop arm rezeroing.
        arm.stopRezeroing();

    }

    // Starts lift rezeroing.
    public void startLiftRezeroing() {

        // Start lift rezeroing.
        lift.startRezeroing();

    }

    // Stops lift rezeroing.
    public void stopLiftRezeroing() {

        // Stop lift rezeroing.
        lift.stopRezeroing();

    }

    // Starts slide rezeroing.
    public void startSlideRezeroing() {

        // Start slide rezeroing.
        slide.startRezeroing();

    }

    // Stops slide rezeroing.
    public void stopSlideRezeroing() {

        // Stop slide rezeroing.
        slide.stopRezeroing();

    }

    // Adds an action to this.
    public void addAction(Action action) {

        // Add the action to this.
        runningActions.add(action);

    }

    // Disables manual driving.
    public void disableManualDriving() {

        // Disable manual driving.
        allowManualDriving = false;

        // Stop the robot.
        drivetrain.stop();

    }

    // Enables manual driving.
    public void enableManualDriving() {

        // Enable manual driving.
        allowManualDriving = true;

    }

}