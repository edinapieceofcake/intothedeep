package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.edina.Libraries.Actions.WaitUntil;

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
    private Drivetrain2 drivetrain;
    private RobotDriver robotDriver;
    private Arm2 arm;
    private Grabber grabber;
    private Extension extension;
    private Lift2 lift;
    private RobotState robotState;
    private CurrentSensor currentSensor;
    private SampleSensor sampleSensor;
    private Light light;

    private double p2mult;

    public RobotHardwareChicago(HardwareMap hw) {
        p2mult = 0.3;

        robotState = new RobotState(hw);

        drivetrain = new Drivetrain2(hw, robotState);
        extension = new Extension(robotState, hw);
        arm = new Arm2(robotState, hw);
        lift = new Lift2(robotState, hw);
        currentSensor = new CurrentSensor(hw);
        sampleSensor = new SampleSensor(hw);
        light = new Light(hw, sampleSensor);
        grabber = new Grabber(robotState, hw);

        robotDriver = new RobotDriver(drivetrain, robotState, runningActions);

        runningActions.add(arm.holdPos());
        runningActions.add(light.makeUpdateAction());
    }

    public void initUpdate() {
        currentSensor.checkForInit();
    }

    public void update(Telemetry telemetry) {
        robotState.update(telemetry);

        TelemetryPacket packet = new TelemetryPacket();
        runningActions.run(packet);

        dash.sendTelemetryPacket(packet);
    }

    public void highBasket() {
        runningActions.add(new ParallelAction(
                extension.moveExtension(0),
                lift.moveLift(Lift2.POS_HIGH_BASKET),
                new SequentialAction(
                        new WaitUntil(() -> robotState.getExtensionPos() < Extension.EXTENSION_RETRACTED_INCHES),
                        new ParallelAction(
                                arm.moveArm(Arm2.POS_HIGH_BASKET),
                                new SequentialAction(
                                        new WaitUntil(() -> robotState.getArmPos() < Arm2.POS_ARM_VERTICAL && robotState.getArmPos() > Arm2.POS_ARM_SCORE_BASKET_MIN),
                                        extension.moveExtension(Extension.POS_HIGH_BASKET)
                                )
                        )
                )
        ));
    }

    public void lowBasket() {
        runningActions.add(new ParallelAction(
                extension.moveExtension(0),
                lift.moveLift(Lift2.POS_LOW_BASKET),
                new SequentialAction(
                        new WaitUntil(() -> robotState.getExtensionPos() < Extension.EXTENSION_RETRACTED_INCHES),
                        new ParallelAction(
                                arm.moveArm(Arm2.POS_LOW_BASKET),
                                new SequentialAction(
                                        new WaitUntil(() -> robotState.getArmPos() < Arm2.POS_ARM_VERTICAL && robotState.getArmPos() > Arm2.POS_ARM_SCORE_BASKET_MIN),
                                        extension.moveExtension(Extension.POS_LOW_BASKET)
                                )
                        )
                )
        ));
    }

    public void toggleClaw() {
        runningActions.add(grabber.toggleClaw());
    }

    public void subMode() {
        Action grabberDown = new SequentialAction(
                new WaitUntil(() -> robotState.getExtensionPos() >= Extension.INIT_EXTENSION_SUB - 1),
                grabber.subMode()
        );

        Action extendInSub = new SequentialAction(
                new WaitUntil(() -> robotState.getArmPos() > Arm2.POS_SPECIMEN),
                new ParallelAction(
                        extension.moveExtension(Extension.INIT_EXTENSION_SUB),
                        grabberDown
                )
        );

        runningActions.add(new ParallelAction(
                grabber.straightWrist(),
                extension.moveExtension(0),
                lift.moveLift(Lift2.POS_BOTTOM),
                new SequentialAction(
                        new WaitUntil(() -> robotState.getExtensionPos() < Extension.EXTENSION_RETRACTED_INCHES),
                        new ParallelAction(
                                arm.moveArm(Arm2.POS_SPECIMEN),
                                extendInSub
                        )
                )
        ));
    }

    public void wallMode() {
        runningActions.add(new ParallelAction(
                grabber.wallMode(),
                extension.moveExtension(0),
                lift.moveLift(Lift2.POS_BOTTOM),
                new SequentialAction(
                        new WaitUntil(() -> robotState.getExtensionPos() < Extension.EXTENSION_RETRACTED_INCHES),
                        arm.moveArm(Arm2.POS_ARM_WALL)
                )
        ));
    }

    public void highSpecimen() {
        runningActions.add(lift.moveLift(13));
        specimenMode();
    }

    public void lowSpecimen() {
        runningActions.add(lift.moveLift(7));
        specimenMode();
    }

    private void specimenMode() {
        runningActions.add(new ParallelAction(
                grabber.specimenMode(),
                extension.moveExtension(0),
                new SequentialAction(
                        new WaitUntil(() -> robotState.getExtensionPos() < Extension.EXTENSION_RETRACTED_INCHES),
                        arm.moveArm(Arm2.POS_SPECIMEN)
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
        runningActions.add(new ParallelAction(
                arm.moveArm(Arm2.POS_GROUND),
                new SequentialAction(
                        new WaitUntil(() -> robotState.getArmPos() >= Arm2.POS_GROUND - 5),
                        grabber.closeClaw(),
                        new WaitForTime(100),
                        grabber.straightWrist(),
                        arm.moveArm(Arm2.POS_SPECIMEN),
                        new SequentialAction(
                                new WaitUntil(() -> robotState.getArmPos() <= Arm2.POS_GROUND - 10),
                                extension.moveExtension(0)
                        )
                )
        ));
    }

    public void addPath(Vector2d[] vectors, double tgtSpeed) {
        robotDriver.addDrivePath(vectors, tgtSpeed);
    }

    public void perpendicularSwivel() {
        runningActions.add(grabber.perpendicularSwivel());
    }

    public void horizontalSwivel() {
        runningActions.add(grabber.horizontalSwivel());
    }

    public boolean armOverSub() {
        return robotState.armOverSub();
    }
}