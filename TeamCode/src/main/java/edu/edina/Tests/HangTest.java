package edu.edina.Tests;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import edu.edina.Libraries.Actions.RaiseLift;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.WaitForTime;

@TeleOp
public class HangTest extends LinearOpMode {
    private boolean up;

    private Gamepad g1 = new Gamepad();
    private Gamepad g2 = new Gamepad();

    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad g1 = new Gamepad();
        Gamepad g2 = new Gamepad();

        up = false;

        ServoImplEx sLeft = hardwareMap.get(ServoImplEx.class, "hang_left");
        ServoImplEx sRight = hardwareMap.get(ServoImplEx.class, "hang_right");

        RobotHardware robotHardware = new RobotHardware(this);
        waitForStart();
        Action action = new SequentialAction();

        waitForStart();

        while (opModeIsActive()) {
            g2.copy(g1);
            g1.copy(gamepad1);

            if (g1.dpad_up && !g2.dpad_up) {
                action = new SequentialAction(
                        new InstantAction(() -> sRight.setPosition(.7)),
                        new InstantAction(() -> sLeft.setPosition(.2)),
                        new WaitForTime(2000),
                        new ParallelAction(
                            new RaiseLift(robotHardware, 14),
                            new InstantAction(sRight::setPwmDisable),
                            new InstantAction(sLeft::setPwmDisable)
                        )
                );
            } else if (g1.dpad_down && !g2.dpad_down) {
                action = new SequentialAction(
                        new WaitForTime(2000),
                        new ParallelAction(
                                new InstantAction(sRight::setPwmDisable),
                                new InstantAction(sLeft::setPwmDisable)
                        ),
                        new RaiseLift(robotHardware, 0)
                );
            }

            robotHardware.drivetrain.update();
            action.run(new TelemetryPacket());
        }
    }
}