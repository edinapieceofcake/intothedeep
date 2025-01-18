package edu.edina.Libraries.Robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.Servo;

public class DualClaw {
    private RobotHardware hw;
    private Servo clawSmall, clawBig;
    private boolean smallOpen;
    private boolean bigOpen;
    public DualClaw(RobotHardware hw) {
        clawSmall = hw.getOpMode().hardwareMap.get(Servo.class, "claw_top");
        clawBig = hw.getOpMode().hardwareMap.get(Servo.class, "claw_bottom");
    }

    public Action closeSmall() {
        return new SequentialAction(
                new InstantAction(() -> clawSmall.setPosition(0.4348))
        );
    }

    public Action closeBig() {
        return new SequentialAction(
                new InstantAction(() -> clawBig.setPosition(0.3917))
        );
    }

    public Action openBoth() {
        return new SequentialAction(
                new InstantAction(() -> clawSmall.setPosition(0.9944)),
                new InstantAction(() -> clawBig.setPosition(0.0000))
        );
    }

    public Action openBig() {
        return new SequentialAction(
                new InstantAction(() -> clawBig.setPosition(0.0000))
        );
    }

    public Action openSmall() {
        return new SequentialAction(
                new InstantAction(() -> clawSmall.setPosition(0.9944))
        );
    }

    public Action toggleSmall() {
        if (smallOpen) {
            smallOpen = false;
            return new SequentialAction(
                    new InstantAction(() -> clawSmall.setPosition(0.4348))

            );

        } else {
            smallOpen = true;
            return new SequentialAction(
                    new InstantAction(() -> clawSmall.setPosition(0.9944))
                    );
        }

    }
    public Action toggleBig() {
        if (bigOpen) {
            bigOpen = false;
            return new SequentialAction(
                    new InstantAction(() -> clawBig.setPosition(0.3917))
            );

        } else {
           bigOpen = true;
            return new SequentialAction(
                    new InstantAction(() -> clawBig.setPosition(0.0000))
            );
        }

    }
}