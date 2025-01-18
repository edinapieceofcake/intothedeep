package edu.edina.Libraries.Robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.Servo;

public class DualClaw {
    private RobotHardware hw;
    private Servo clawTop, clawBottom;

    public DualClaw(RobotHardware hw) {
        clawTop = hw.getOpMode().hardwareMap.get(Servo.class, "claw_top");
        clawBottom = hw.getOpMode().hardwareMap.get(Servo.class, "claw_bottom");
    }

    public Action closeTop() {
        return new SequentialAction(
                new InstantAction(() -> clawTop.setPosition(0.4348)),
                new WaitForTime(100)
        );
    }

    public Action closeBottom() {
        return new SequentialAction(
                new InstantAction(() -> clawBottom.setPosition(0.3917)),
                new WaitForTime(100)
        );
    }

    public Action openBoth() {
        return new SequentialAction(
                new InstantAction(() -> clawTop.setPosition(0.9944)),
                new InstantAction(() -> clawBottom.setPosition(0.0000)),
                new WaitForTime(100)
        );
    }
}