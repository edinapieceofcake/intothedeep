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

    public void closeSmall() {
        clawSmall.setPosition(0.4348);
    }

    public void closeBig() {
        clawBig.setPosition(0.3917);
    }

    public Action openBoth() {
        return new SequentialAction(
                new InstantAction(() -> clawSmall.setPosition(0.9944)),
                new InstantAction(() -> clawBig.setPosition(0.0000))
        );
    }

    public void openBig() {
        clawBig.setPosition(0.0000);
    }

    public void openSmall() {
        clawSmall.setPosition(0.9944);
    }

    public void toggleSmall() {
        if (smallOpen) {
            smallOpen = false;
            closeSmall();

        } else {
            smallOpen = true;
           openSmall();
        }

    }
    public void toggleBig() {
        if (bigOpen) {
            bigOpen = false;
           closeBig();

        } else {
           bigOpen = true;
           openBig();
        }

    }
}