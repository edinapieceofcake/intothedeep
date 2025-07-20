package edu.edina.Libraries.Robot;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

import edu.edina.Libraries.Actions.LogAction;
import edu.edina.Libraries.Actions.WaitUntil;
import edu.edina.Libraries.PurePursuit.BrakeAction;
import edu.edina.Libraries.PurePursuit.Path;
import edu.edina.Libraries.PurePursuit.PurePursuitAction;

// Control hub:
//   Motor 0: extension_motor
//   Motor 1: left_front_drive
//   Motor 2: left_back_drive
//   Motor 3: left_lift_motor
//   Servo 3: swivel
//   Servo 4: wrist
//   I2C Bus 0:
//     imu
//     sensor_ina260
//   I2C Bus 1:
//     neopixel_driver
//   I2C Bus 2:
//     sensor_otos
//   I2C Bus 3:
//     sensor_color
// Expansion hub:
//   Motor 0: right_lift_motor
//   Motor 1: right_back_drive
//   Motor 2: right_front_drive
//   Motor 3: arm_motor
//   Servo 3: claw_bottom
//   I2C 1:
//     distance_left
//   I2C 2:
//     distance_right

public class RobotHardwareChicago {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private ActionList runningActions = new ActionList();
    private Drivetrain drivetrain;
    private RobotDriver robotDriver;
    private Arm arm;
    private Grabber grabber;
    private Extension extension;
    private Lift lift;
    private RobotState robotState;
    private CurrentSensor currentSensor;
    private SampleSensor sampleSensor;
    private Light light;
    private VisionPortal portal;
    private ColorBlobLocatorProcessorTwo procYellow;

    private double p2mult;

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    public RobotState getRobotState() {
        return robotState;
    }

    public RobotHardwareChicago(HardwareMap hw) {
        this(hw, RobotState.getLastPose2d());
    }

    public Arm getArm() {
        return arm;
    }

    public Extension getExtension() {
        return extension;
    }

    public Lift getLift() {
        return lift;
    }

    public RobotHardwareChicago(HardwareMap hw, Pose2d initPose) {
        p2mult = 0.3;

        robotState = new RobotState(hw, initPose);

        drivetrain = new Drivetrain(hw, robotState);
        extension = new Extension(robotState, hw);
        arm = new Arm(robotState, hw);
        lift = new Lift(robotState, hw);
        currentSensor = new CurrentSensor(hw);
        sampleSensor = new SampleSensor(hw);
        light = new Light(hw, sampleSensor);
        grabber = new Grabber(robotState, hw);

        robotDriver = new RobotDriver(drivetrain, robotState, runningActions);

        procYellow = new ColorBlobLocatorProcessorTwo.Builder()
                .setTargetColorRange(ColorRangeTwo.YELLOW)
                .setContourMode(ColorBlobLocatorProcessorTwo.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegionTwo.asUnityCenterCoordinates(-1, 0, 1, -1))
                .setDrawContours(true)
                .setBlurSize(4)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(160, 120))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .addProcessor(procYellow)
                .build();

        runningActions.add(arm.holdPos());
        runningActions.add(light.makeUpdateAction());
    }

    public void enableYellow() {
        portal.setProcessorEnabled(procYellow, true);
    }

    public void initUpdate(Telemetry telemetry) {
        robotState.update(telemetry);
        currentSensor.checkForInit();

        TelemetryPacket packet = new TelemetryPacket();

        for (String p : new String[]{"arm-mc", "arm-spp"}) {
            packet.put(p, 0.0);
        }

        dash.sendTelemetryPacket(packet);
    }

    public void update(Telemetry telemetry) {
        robotState.update(telemetry);

        TelemetryPacket packet = new TelemetryPacket();
        runningActions.run(packet);

        dash.sendTelemetryPacket(packet);
    }

    public void highBasketMode() {
        addPrimaryAction(
                new ParallelAction(
                        new LogAction("highBasket", "start"),
                        grabber.horizontalSwivel(),
                        extension.moveAndHold(0),
                        lift.moveAndHold(Lift.POS_HIGH_BASKET),
                        new SequentialAction(
                                new WaitUntil("high basket/ext retract", () -> robotState.getExtensionPos() < Extension.EXTENSION_RETRACTED_INCHES),
                                new LogAction("highBasket", "done waiting for ext"),
                                new ParallelAction(
                                        arm.moveAndHold(Arm.POS_HIGH_BASKET),
                                        new SequentialAction(
                                                new WaitUntil("high basket/arm retract", () -> robotState.getArmPos() < Arm.POS_ARM_VERTICAL && robotState.getArmPos() > Arm.POS_ARM_SCORE_BASKET_MIN && robotState.getArmSpeed() < 1),
                                                new LogAction("highBasket", "done waiting for arm"),
                                                extension.moveAndHold(Extension.POS_HIGH_BASKET)
                                        ),
                                        new SequentialAction(
                                                new WaitUntil("high basket/ext", () -> robotState.getExtensionPos() > Extension.POS_HIGH_BASKET - 2),
                                                grabber.highBasket()
                                        )
                                )
                        )
                ));
    }

    public Action makeHighBasketRearAction() {
        return new ParallelAction(
                new LogAction("highBasketRear", "start"),
                grabber.swivelEnd(),
                grabber.straightWrist(),
                extension.moveAndHold(0),
                lift.moveAndHold(Lift.POS_HIGH_BASKET),
                new SequentialAction(
                        new WaitUntil("high basket rear/extension retract", () -> robotState.getExtensionPos() < Extension.EXTENSION_RETRACTED_INCHES),
                        new LogAction("highBasketRear", "done waiting for ext"),
                        new ParallelAction(
                                arm.moveAndHold(Arm.POS_ARM_VERTICAL),
                                new SequentialAction(
                                        new WaitUntil("high basket/arm past vertical", () -> robotState.getArmPos() < Arm.POS_ARM_VERTICAL + 10 && robotState.getArmPos() > Arm.POS_ARM_VERTICAL - 10),
                                        new LogAction("highBasketRear", "done waiting for arm"),
                                        new ParallelAction(
                                                extension.moveAndHold(Extension.POS_HIGH_BASKET),
                                                new SequentialAction(
                                                        new WaitUntil("high basket/ext", () -> robotState.getExtensionPos() > Extension.POS_HIGH_BASKET - 2),
                                                        new ParallelAction(
                                                                grabber.sampleRear(),
                                                                new LogAction("highBasketRear", "dropped"),
                                                                release()
                                                        )
                                                )
                                        )
                                )
                        )
                )
        );
    }

    public void lowBasketMode() {
        addPrimaryAction(
                new ParallelAction(
                        new LogAction("lowBasket", "start"),
                        grabber.horizontalSwivel(),
                        extension.moveAndHold(0),
                        lift.moveAndHold(Lift.POS_LOW_BASKET),
                        new SequentialAction(
                                new WaitUntil("low basket/ext retract", () -> robotState.getExtensionPos() < Extension.EXTENSION_RETRACTED_INCHES),
                                new LogAction("lowBasket", "done waiting for ext"),
                                new ParallelAction(
                                        arm.moveAndHold(Arm.POS_LOW_BASKET),
                                        new SequentialAction(
                                                new WaitUntil("low basket/arm past vertial", () -> robotState.getArmPos() < Arm.POS_ARM_VERTICAL && robotState.getArmPos() > Arm.POS_ARM_SCORE_BASKET_MIN && Math.abs(robotState.getArmSpeed()) < 5),
                                                new LogAction("lowBasket", "done waiting for arm"),
                                                extension.moveAndHold(Extension.POS_LOW_BASKET)
                                        )
                                )
                        )
                ));
    }

    public void toggleClaw() {
        runningActions.add(grabber.toggleClaw());
    }

    public void openClaw() {
        runningActions.add(grabber.openClaw());
    }

    public void closeClaw() {
        runningActions.add(grabber.closeClaw());
    }

    public void subMode() {
        Action grabberDown = new SequentialAction(
                new WaitUntil("sub/ext", () -> robotState.getExtensionPos() >= Extension.INIT_EXTENSION_SUB - 1),
                new LogAction("subMode", "done waiting for arm (2)"),
                grabber.subMode()
        );

        Action extendInSub = new SequentialAction(
                new WaitUntil("sub/arm up", () -> arm.at(Arm.POS_SUBMERSIBLE, 10)),
                new LogAction("subMode", "done waiting for arm"),
                new ParallelAction(
                        extension.moveAndHold(Extension.INIT_EXTENSION_SUB),
                        grabberDown
                )
        );

        addPrimaryAction(
                new ParallelAction(
                        new LogAction("subMode", "start"),
                        grabber.straightWrist(),
                        extension.moveAndHold(0),
                        lift.moveAndHold(Lift.POS_BOTTOM),
                        new SequentialAction(
                                new WaitUntil("sub/ext retract", () -> robotState.getExtensionPos() < Extension.EXTENSION_RETRACTED_INCHES),
                                new LogAction("subMode", "done waiting for ext"),
                                new ParallelAction(
                                        arm.moveAndHold(Arm.POS_SUBMERSIBLE),
                                        extendInSub
                                )
                        )
                ));
    }

    public Action makeWallModeAction() {
        return new ParallelAction(
                new LogAction("wallMode", "start"),
                grabber.wallMode(),
                extension.moveAndHold(0),
                lift.moveAndHold(Lift.POS_BOTTOM),
                new SequentialAction(
                        new WaitUntil("wall/ext retract", () -> robotState.getExtensionPos() < Extension.EXTENSION_RETRACTED_INCHES),
                        new LogAction("wallMode", "done waiting for ext"),
                        new ParallelAction(
                                arm.moveAndHold(Arm.POS_ARM_WALL),
                                extension.makeResetAction()
                        )
                )
        );
    }

    public Action makeWallAction() {
        return new ParallelAction(
                new LogAction("wallMode", "start"),
                grabber.wallMode(),
                extension.moveAndHold(0),
                lift.moveAndHold(Lift.POS_BOTTOM),
                new SequentialAction(
                        new WaitUntil("wall action/ext retract", () -> robotState.getExtensionPos() < Extension.EXTENSION_RETRACTED_INCHES),
                        new LogAction("wallMode", "done waiting for ext"),
                        new ParallelAction(
                                arm.moveAndHold(Arm.POS_ARM_WALL),
                                extension.makeResetAction(),
                                new SequentialAction(
                                        new WaitUntil("wall action/arm at wall", () -> arm.at(Arm.POS_ARM_WALL, 5)),
                                        arm.release(),
                                        extension.release(),
                                        lift.release()
                                )
                        )
                )
        );
    }

    public void wallMode() {
        addPrimaryAction(makeWallModeAction());
    }

    public void highSpecimen() {
        addPrimaryAction(
                new ParallelAction(
                        lift.moveAndHold(12),
                        new LogAction("highSpecimen", "start"),
                        new SequentialAction(
                                new WaitForTime(500),
                                grabber.specimenMode()
                        ),
                        extension.moveAndHold(0),
                        new SequentialAction(
                                new WaitUntil("high specimen/ext retract", () -> robotState.getExtensionPos() < Extension.EXTENSION_RETRACTED_INCHES),
                                new LogAction("highSpecimen", "done waiting for ext"),
                                arm.moveAndHold(Arm.POS_SPECIMEN)
                        ),
                        new SequentialAction(
                                new WaitUntil("high specimen/arm past vert", () -> robotState.getArmPos() > Arm.POS_ARM_VERTICAL),
                                new LogAction("highSpecimen", "done waiting for arm"),
                                extension.moveAndHold(Extension.POS_CHAMBER)
                        )
                ));
    }

    public void lowSpecimen() {
        addPrimaryAction(
                new ParallelAction(
                        lift.moveAndHold(0),
                        new LogAction("lowSpecimen", "start"),
                        new SequentialAction(
                                new WaitForTime(500),
                                grabber.specimenMode()
                        ),
                        extension.moveAndHold(0),
                        new SequentialAction(
                                new WaitUntil("low specimen/ext retract", () -> robotState.getExtensionPos() < Extension.EXTENSION_RETRACTED_INCHES),
                                new LogAction("lowSpecimen", "done waiting for ext"),
                                arm.moveAndHold(Arm.POS_LOW_SPECIMEN)
                        ),
                        new SequentialAction(
                                new WaitUntil("low specimen/arm past vertical", () -> robotState.getArmPos() > Arm.POS_ARM_VERTICAL),
                                extension.moveAndHold(Extension.POS_CHAMBER)
                        )
                ));
    }

    public void extend(double y) {
        if (Math.abs(y) > 0.1) {
            runningActions.add(extension.manuallyAdjust(y));
        }
    }

    public void drive(Gamepad gamepad1, Gamepad gamepad2) {
        drivetrain.update2(gamepad1, gamepad2, p2mult);
    }

    public void intake() {
        addPrimaryAction(
                new ParallelAction(
                        new LogAction("intake", "start"),
                        arm.constantPower(Arm.INTAKE_POWER),
                        new SequentialAction(
                                new WaitUntil("intake/arm at ground", () -> robotState.getArmPos() >= Arm.POS_GROUND - 5),
                                new LogAction("intake", "done waiting for arm"),
                                grabber.closeClaw(),
                                new WaitForTime(120),
                                new ParallelAction(
                                        arm.moveAndHold(Arm.POS_SUBMERSIBLE),
                                        new SequentialAction(
                                                new WaitUntil("intake/arm above ground", () -> robotState.getArmPos() <= Arm.POS_GROUND - 10),
                                                new LogAction("intake", "done waiting for arm"),
                                                grabber.straightWrist(),
                                                extension.moveAndHold(0)
                                        )
                                )
                        )
                ));
    }

    public Action waitForGroundMode(String prefix) {
        return new WaitUntil(String.format("%sground mode", prefix),
                () -> arm.at(Arm.POS_GROUND_FRONT, 10)
                        && extension.at(Extension.EXTENSION_RETRACTED_INCHES, 2)
                        && lift.at(Lift.POS_BOTTOM, 1)
        );
    }

    public Action makeGroundModeAction() {
        return new ParallelAction(
                arm.moveAndHold(Arm.POS_GROUND_FRONT),
                extension.moveAndHold(Extension.EXTENSION_RETRACTED_INCHES),
                lift.moveAndHold(Lift.POS_BOTTOM),
                grabber.groundWrist()
        );
    }

    public Action addPath(Path path) {
        return robotDriver.addDrivePath(path);
    }

    public void perpendicularSwivel() {
        runningActions.add(grabber.perpendicularSwivel());
    }

    public void horizontalSwivel() {
        runningActions.add(grabber.horizontalSwivel());
    }

    public void halfSwivel() {
        runningActions.add(grabber.halfSwivel());
    }

    public boolean armOverSub() {
        return robotState.armOverSub();
    }

    private Action primaryAction = null;

    public void addPrimaryAction(Action a) {
        runningActions.remove(primaryAction);
        primaryAction = a;
        runningActions.add(primaryAction);
    }

    public void brake() {
        drivetrain.stop();
    }

    public void calibrateIMU() {
        robotState.calibrateIMU();
    }

    public void addAction(Action a) {
        runningActions.add(a);
    }

    public void addSampleTelemetry(Telemetry telemetry) {
        List<ColorBlobLocatorProcessorTwo.Blob> blobs = procYellow.getBlobs();
        for (int i = 0; i < blobs.size(); i++) {
            ColorBlobLocatorProcessorTwo.Blob blob = blobs.get(i);
            telemetry.addData(String.format("blob %d", i),
                    "area = %d, AR = %.1f, d = %.1f", blob.getContourArea(),
                    blob.getAspectRatio(),
                    blob.getDensity());
        }

        SampleLocation loc = getSampleLocation();
        if (loc != null)
            telemetry.addData("sample loc", loc);
    }

    public SampleLocation getSampleLocation() {
        List<ColorBlobLocatorProcessorTwo.Blob> blobs = procYellow.getBlobs();
        ColorBlobLocatorProcessorTwo.Util.filterByArea(SampleLocator.MIN_AREA, SampleLocator.MAX_AREA, blobs);
        ColorBlobLocatorProcessorTwo.Util.filterByDensity(SampleLocator.MIN_DENSITY, 1, blobs);
        ColorBlobLocatorProcessorTwo.Util.filterByAspectRatio(
                SampleLocator.MIN_ASPECT_RATIO, 1 / SampleLocator.MIN_ASPECT_RATIO, blobs);

        if (blobs.isEmpty())
            return null;


        ColorBlobLocatorProcessorTwo.Blob bestBlob = null;
        for (ColorBlobLocatorProcessorTwo.Blob b : blobs) {
            RobotLog.ii("sampleLocation", "blob %.1f", b.getBoxFit().center.x);

            if (bestBlob == null)
                bestBlob = b;
            else if (dist(b) < dist(bestBlob))
                bestBlob = b;
        }

        double loc = bestBlob.getBoxFit().center.x - 80;
        RobotLog.ii("sampleLocation", "blob %.1f, sample loc %.1f", bestBlob.getBoxFit().center.x, loc);

        return new SampleLocation(loc);
    }

    private double dist(ColorBlobLocatorProcessorTwo.Blob b) {
        return Math.abs(80 - b.getBoxFit().center.x);
    }

    public void coast() {
        drivetrain.coast();
    }

    public Action sequencePath(Path path, double minSpeed) {
        return new SequentialAction(
                new LogAction("sequencePath", path.routeString()),
                new LogAction("sequencePath", "fast drive"),
                new PurePursuitAction(path, drivetrain, robotState, true),
                new LogAction("sequencePath", "braking"),
                new BrakeAction(robotState, drivetrain, minSpeed),
                new LogAction("sequencePath", "slow drive"),
                new PurePursuitAction(path, drivetrain, robotState, false),
                new LogAction("sequencePath", "done"),
                new InstantAction(this::coast)
        );
    }

    public void wristUp() {
        grabber.groundWrist();
    }

    public void groundIntake() {
        addPrimaryAction(makeGroundIntakeModeAction());
    }

    public Action makeGroundIntakeModeAction() {
        return new SequentialAction(
                new ParallelAction(
                        grabber.openClaw(),
                        extension.moveAndHold(Extension.POS_GROUND_INTAKE),
                        new LogAction("gintake", "start"),
                        arm.moveAndHold(Arm.POS_GROUND_FRONT),
                        new SequentialAction(
                                new WaitUntil("ground intake/arm at ground", () -> armAt(Arm.POS_GROUND_FRONT, 12)),
                                new LogAction("gintake", "done waiting for arm"),
                                grabber.closeClaw(),
                                new WaitForTime(100),
                                new ParallelAction(
                                        arm.moveAndHold(Arm.POS_ARM_WALL),
                                        extension.moveAndHold(Extension.EXTENSION_RETRACTED_INCHES),
                                        new SequentialAction(
                                                new WaitUntil("ground intake/arm above ground", () -> robotState.getArmPos() >= Arm.POS_ARM_WALL - 10),
                                                new LogAction("gintake", "done waiting for arm (2)"),
                                                release(),
                                                new LogAction("gintake", "released")
                                        )
                                )
                        )
                ),
                new LogAction("intake", "all done")
        );
    }

    public boolean armAt(double pos, double tol) {
        return arm.at(pos, tol);
    }

    public Action release() {
        return new ParallelAction(
                arm.release(),
                extension.release(),
                lift.release()
        );
    }

    public Action makeBrakeAction(double minSpeed) {
        return new BrakeAction(robotState, drivetrain, minSpeed);
    }

    public Grabber getGrabber() {
        return grabber;
    }
}