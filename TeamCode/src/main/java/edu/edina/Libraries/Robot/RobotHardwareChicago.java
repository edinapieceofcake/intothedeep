package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

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
    private List<Action> runningActions = new ArrayList<>();
    private Drivetrain drivetrain;
    private Arm2 arm;
    private Grabber grabber;
    private Extension extension;
    private Lift2 lift;
    private RobotState robotState;
    private TelemetryPacket packet;
    private CurrentSensor currentSensor;
    private SampleSensor sampleSensor;
    private Light light;

    public RobotHardwareChicago(HardwareMap hw) {
        packet = new TelemetryPacket();

        robotState = new RobotState(hw);

        drivetrain = new Drivetrain(hw);
        extension = new Extension(robotState, hw);
        arm = new Arm2(robotState, hw);
        lift = new Lift2(robotState, hw);
        currentSensor = new CurrentSensor(hw);
        sampleSensor = new SampleSensor(hw);
        light = new Light(hw, sampleSensor);
        grabber = new Grabber(robotState, hw);

        runningActions.add(arm.holdPos());
        runningActions.add(light.makeUpdateAction());
    }

    public void initUpdate() {
        currentSensor.checkForInit();
    }

    public void update(Telemetry telemetry) {
        robotState.update(telemetry);

        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
    }

    public void highBasket() {
        runningActions.add(new ParallelAction(
                lift.moveLift(12),
                extension.moveExtension(10),
                arm.moveArm(60)
        ));
    }

    public void toggleClaw() {
        runningActions.add(grabber.toggleClaw());
    }

    public void subMode() {
        runningActions.add(grabber.subMode());
        runningActions.add(extension.moveExtension(5));
        runningActions.add(lift.moveLift(0));
        runningActions.add(arm.moveArm(180));
    }

    public void wallMode() {
        runningActions.add(grabber.wallMode());
        runningActions.add(extension.moveExtension(0));
        runningActions.add(lift.moveLift(0));
        runningActions.add(arm.moveArm(0));
    }

    public void highSpecimen() {
        runningActions.add(lift.moveLift(8));
        specimenMode();
    }

    public void lowSpecimen() {
        runningActions.add(lift.moveLift(4));
        specimenMode();
    }

    public void specimenMode() {
        runningActions.add(grabber.specimenMode());
        runningActions.add(extension.moveExtension(0));
        runningActions.add(arm.moveArm(180));
    }

    public void extend(double y) {
        if (Math.abs(y) < 0.1 && !extension.hasCurrentAction()) {
            runningActions.add(extension.holdPos());
        } else {
            extension.cancelAction();
            extension.setPower(y);
        }
    }

    public void drive(Gamepad gamepad) {
        drivetrain.update2(gamepad);
    }

    public void intake() {
        runningActions.add(new SequentialAction(
                arm.moveArm(200),
                new WaitForTime(500),
                grabber.closeClaw(),
                new WaitForTime(200),
                extension.moveExtension(0)
        ));
    }
}