package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class DualClaw {
    public static double SMALL_OPEN = 0.75;
    public static double BIG_OPEN = 0;
    public static double SMALL_CLOSE = 0.22;
    public static double BIG_CLOSE = 0.3917;
    private RobotHardware hw;
    private Servo clawSmall, clawBig;
    private boolean smallOpen;
    private boolean bigOpen;
    public DualClaw(RobotHardware hw) {
        clawSmall = hw.getOpMode().hardwareMap.get(Servo.class, "claw_top");
        clawBig = hw.getOpMode().hardwareMap.get(Servo.class, "claw_bottom");
    }

    public void closeSmall() {
        clawSmall.setPosition(SMALL_CLOSE);
    }

    public void closeBig() {
        clawBig.setPosition(BIG_CLOSE);
    }

    public Action openBoth() {
        return new SequentialAction(
                new InstantAction(() -> clawSmall.setPosition(SMALL_OPEN)),
                new InstantAction(() -> clawBig.setPosition(BIG_OPEN))
        );
    }

    public void openBig() {
        clawBig.setPosition(BIG_OPEN);
    }

    public void openSmall() {
        clawSmall.setPosition(SMALL_OPEN);
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