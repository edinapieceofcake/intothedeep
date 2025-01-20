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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

import edu.edina.Libraries.RoadRunner.Localizer;
import edu.edina.Libraries.RoadRunner.MecanumDrive;
import edu.edina.Libraries.RoadRunner.ThreeDeadWheelLocalizer;
import edu.edina.OpModes.Autonomous.AutoSample;
import edu.edina.OpModes.Autonomous.AutoSpecimen;

@Config
public class RobotHardware implements DrivingRobotHardware {
    /*
    Control Hub Portal
        Control Hub
            Motors
                0 - GoBILDA 5201 series - left_back_drive (encoder port returns 0 and -1)
                1 - GoBILDA 5201 series - left_front_drive (has front odometry encoder)
                front_encoder (is front odometry encoder)
                2 - GoBILDA 5201 series - extension_motor
                3 - GoBILDA 5201 series - left_lift_motor (has left lift encoder)
            Servos

                wrist_left
                3 - Servo - swivel
                4 - Servo - wrist
                5 - Servo - claw_top
            Digital Devices
                5 - REV Touch Sensor - arm_touch_front
            I2C Bus 2
                0 - Neopixel Driver - neopixel_driver
            I2C Bus 3
                0 - REV Color/Range Sensor - sensor_color
            Analog
                0 - Analog Input - wrist_left_encoder
                1 - Analog Input - wrist_right_encoder
        Expansion Hub 2
            Motors
                0 - GoBILDA 5201 series - right_lift_motor (has right lift encoder)
                1 - GoBILDA 5201 series - right_back_drive (has right odometry encoder)
                2 - GoBILDA 5201 series - right_front_drive (encoder port has bent pin)
                3 - GoBILDA 5201 series - arm_motor (has through bore encoder)
            Servos
                3 - Servo - claw_bottom
            Digital Devices
               1 - REV Touch Sensor - lift_touch
               3 - REV Touch Sensor - arm_touch_back
            Analog (don't knoe it it's here)
                0 - Analog Input - slide_encoder
            I2C Bus 0
                0 - REV Internal IMU (BNO055) - imu
            I2C Bus 1
                0 - REV 2M Distance Sensor - distance_left
            I2C Bus 2
                0 - REV 2M Distance Sensor - distance_right
    */

    // Inches to back up when scoring a specimen
    public static int SCORE_SPECIMEN_BACKUP_INCHES = 7;

    private final LinearOpMode opMode;
    private final Slide2 slide;
    public final IMU imu;
    private final Odometry teleOpOdometry;
    public final Localizer odometry;
    public final MecanumDrive drive;
    public final VoltageSensor voltageSensor;
    public final Drivetrain drivetrain;
    private final Wrist wrist;
    private final DualClaw claw;
    private final Swivel swivel;
    private final Arm arm;
    private final Lift lift;
    private Light light;
    private SampleSensor sampleSensor;
    private boolean turtleMode;
    private int beepSoundId;
    private List<Action> runningActions = new ArrayList<>();
    private FtcDashboard dashboard;
    private boolean allowManualDriving = true;
    private boolean debugging;
    public RearDistanceSensor distanceSensors;
    private DcMotorEx extension;

    public RobotHardware(LinearOpMode opMode) throws InterruptedException {

        // Remember the op mode.
        this.opMode = opMode;

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

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
        claw = new DualClaw(this);

        // Initialize the lift.
        lift = new Lift(this);

        // Initialize the slide.
        slide = new Slide2(this);

        // Initialize the wrist.
        wrist = new Wrist(hardwareMap, opMode.telemetry);

        // Initialize the swivel.
        swivel = new Swivel(hardwareMap, opMode.telemetry);

        // Initialize the drivetrain.
        drivetrain = new Drivetrain(opMode);

//        failsafe = new BoundingBoxFailsafe(wrist, arm, lift, slide);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        odometry = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        distanceSensors = new RearDistanceSensor(hardwareMap);

        teleOpOdometry = new LocalizerOdometry(odometry);

        extension = hardwareMap.get(DcMotorEx.class, "extension_motor");
    }

    public void initializeLights() {
        sampleSensor = new SampleSensor(opMode.hardwareMap);
        light = new Light(opMode.hardwareMap, sampleSensor);
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

    // Sets the wrist to wall position.
    public void setWristWallPosition() {

        // Sets the wrist to wall position.
        wrist.setWallPosition();

    }

    // Sets the wrist to high basket position.
    public void setWristHighBasketPosition() {

        // Sets the wrist to high basket position.
        wrist.setHighBasketPosition();

    }

    // Lowers the wrist.
    public void lowerWrist() {

        // Lowers the wrist.
        wrist.lower();

    }

    // Angles the wrist.
//    public void angleWrist() {
//
//        // Angles the wrist.
//        wrist.angle();
//
//    }

    // Sets the swivel to horizontal.
    public void swivelSetHorizontal() {

        // Sets the swivel to horizontal.
        swivel.setHorizontal();

    }

    // Sets the swivel to vertical.
    public void swivelSetVertical() {

        // Sets the swivel to horizontal.
        swivel.setVertical();

    }

    // Sets the swivel to clip.
    public void swivelSetClip() {

        // Construct an action to set swivel to clip position after a delay.
        swivel.setClip();

    }

    // Sets the swivel to clip without a delay.
    public void swivelSetClipNoDelay() {

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

        drivetrain.setAutoTurtleMode(lift.isRaised() && !debugging);

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
//        claw.update();

        // If manual driving is allowed...
        if (allowManualDriving) {

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
        if (light != null)
            light.update(false, true);

        runActions();
    }

    public void runActions() {
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
//        claw.toggle();

    }

    // Opens the claw.
    public void openClaw() {

        // Open the claw.
        // claw.open();

    }

    public void openBothClaws() {

        // Open the claw.
        runningActions.add(claw.openBoth());
    }

    public void openBigClaw() {
        claw.openBig();
    }

    public void openSmallClaw() {
        claw.openSmall();
    }

    public void closeSmallClaw() {

        claw.closeSmall();
    }

    public void closeBigClaw() {
        claw.closeBig();
    }

    public void toggleSmallClaw() {
        claw.toggleSmall();
    }

    public void toggleBigClaw() {
        claw.toggleBig();
    }


    // Closes the claw.
    public void closeClaw() {

        // Close the claw.
//        claw.close();

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
//        slide.setPosition(position);

    }

    // Retracts the slide.
    public void retractSlide() {

        // Retract the slide.
//        slide.retract();

    }

    // Extends the slide.
    public void extendSlide() {

        // Extend the slide.
//        slide.extend();

    }

    // Sets the minimum extension.
    public void setMinimumExtension() {

        // Set the minimum extension.
        slide.setMinimumExtension();

    }


    // Sets the high basket extension.
    public void setMaximumExtension() {

        // Set the high basket extension.
        slide.setMaximumExtension();

    }
    public void setInitializeExtension() {

        // Set the high basket extension.
        slide.setInitializeExtension();

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

    // Moves the arm to the wall position.
    public void setArmWallPosition() {

        // Construct an action to move the arm to the wall position.
        Action action = new MoveArm(this, Arm.WALL_POSITION, true);

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

    // Moves the lift to the chamber position.
    public void setLiftChamberPosition() {

        // Move the lift to the chamber position.
        lift.setChamberPosition();

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
        return false;

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
        return 0;

    }

    public void setWristHighChamberPosition() {
        wrist.setHighChamberPosition();
    }


    public BoundingBoxFailsafe getFailsafe() {
        return null;
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
                AutoSpecimen.scoreSpecimen(backup, this),
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

    // Determines whether the lift is in the chamber position.
    public boolean isLiftInChamberPosition() {

        // Return indicating if the lift is in the chamber position.
        return lift.isInChamberPosition();

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
        return false;

    }

    // Determines whether the slide is fully extended.
    public boolean isSlideFullyExtended() {

        // Return indicating if the slide is fully extended.
        return false;

    }

    // Gets the lift's current position.
    public int getCurrentLiftPosition() {

        // Return the lift's current position.
        return (int) lift.getPosition();

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
//        slide.startRezeroing();

    }

    // Stops slide rezeroing.
    public void stopSlideRezeroing() {

        // Stop slide rezeroing.
//        slide.stopRezeroing();

    }

    // Adds an action to this.
    public void addAction(Action action) {

        // Add the action to this.
        runningActions.add(action);

    }

    public void stopDrivetrain() {
        drivetrain.stop();
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

    // Sets debugging.
    public void setDebugging(boolean debugging) {
        this.debugging = debugging;
    }

    @Override
    public Odometry getOdometry() {
        return teleOpOdometry;
    }

    public Arm getArm() {
        return arm;
    }

    public Wrist getWrist() {
        return wrist;
    }

    public DualClaw getDualClaw() {
        return claw;
    }

    public Swivel getSwivel() {
        return swivel;
    }

    public DcMotorEx getExtension() {
        return extension;
    }

    @Override
    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    @Override
    public VoltageSensor getVoltageSensor() {
        return voltageSensor;
    }

    public Lift getLift() {
        return lift;
    }
}