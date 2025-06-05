package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.Action;

@Config
public class Grabber {
    private Servo claw, wrist, swivel;
    private RobotState robotState;

    //claw
    public static double CLOSED_POS = 0.576;
    public static double OPEN_POS = 0.53;

    //wrist
    public static double INTAKE_POSITION = 0.4;
    public static double SPECIMEN_POSITION = 1;
    public static double WALL_POSITION = (INTAKE_POSITION + SPECIMEN_POSITION) / 2;

    //swivel
    public static double HORIZONTAL_POS = 0;
    public static double VERTICAL_POS = 0.576;

    private boolean open;

    public Grabber(RobotState robotState, HardwareMap hw) {
        claw = hw.get(Servo.class, "claw_bottom");
        wrist = hw.get(Servo.class, "wrist");
        swivel = hw.get(Servo.class, "swivel");

        open = false;

        this.robotState = robotState;
    }

    //update with tracking and cancelable etc.
    public Action toggleClaw() {
        double pos = open ? CLOSED_POS : OPEN_POS;
        open = !open;
        return new InstantAction(() -> claw.setPosition(pos));
    }

    public Action openClaw() {
        return new InstantAction(() -> claw.setPosition(OPEN_POS));
    }

    public Action closeClaw() {
        return new InstantAction(() -> claw.setPosition(CLOSED_POS));
    }

    public Action subMode() {
        return new InstantAction(() -> {
            wrist.setPosition(INTAKE_POSITION);
            claw.setPosition(OPEN_POS);
            swivel.setPosition(HORIZONTAL_POS);
        });
    }

    public Action specimenMode() {
        return new InstantAction(() -> {
            wrist.setPosition(SPECIMEN_POSITION);
            claw.setPosition(CLOSED_POS);
            swivel.setPosition(HORIZONTAL_POS);
        });
    }

    public Action wallMode() {
        return new InstantAction(() -> {
            wrist.setPosition(WALL_POSITION);
            claw.setPosition(OPEN_POS);
            swivel.setPosition(HORIZONTAL_POS);
        });
    }
}