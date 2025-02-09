package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class DualClaw {
    public static double BIG_CLOSE = .75;
    public static double BIG_OPEN = .28;
    public static double SMALL_CLOSE = .23;
    public static double SMALL_OPEN = .78;
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
        smallOpen = false;
    }

    public void closeBig() {
        clawBig.setPosition(BIG_CLOSE);
        bigOpen = false;
    }

    public Action openBoth() {
        return new SequentialAction(
                new InstantAction(() -> openSmall()),
                new InstantAction(() -> openBig())
        );
    }

    public void openBig() {
        clawBig.setPosition(BIG_OPEN);
        bigOpen = true;
    }

    public void openSmall() {
        clawSmall.setPosition(SMALL_OPEN);
        smallOpen = true;
    }

    public void toggleSmall() {
        if (smallOpen) {
            closeSmall();
        } else {
            openSmall();
        }
    }

    public void toggleBig() {
        if (bigOpen) {
            closeBig();
        } else {
            openBig();
        }
    }

    public boolean isBigOpen() {
        return bigOpen;
    }

    public void turnOff() {
        ((ServoImplEx) clawSmall).setPwmDisable();
        ((ServoImplEx) clawBig).setPwmDisable();
    }

    public void turnOn() {
        ((ServoImplEx) clawSmall).setPwmEnable();
        ((ServoImplEx) clawBig).setPwmEnable();
    }
}