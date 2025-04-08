package edu.edina.Libraries.Actions;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ButtonCondition implements Condition {
    private final Button button;
    private final OpMode opMode;

    public ButtonCondition(Button button, OpMode opMode) {
        this.button = button;
        this.opMode = opMode;
    }

    @Override
    public boolean run() {
        Gamepad g1 = opMode.gamepad1;
        Gamepad g2 = opMode.gamepad2;

        switch (button) {
            case A1:
                return g1.a;
            case B1:
                return g1.b;
            case X1:
                return g1.x;
            case Y1:
                return g1.y;
            case Right1:
                return g1.dpad_right;
            case Down1:
                return g1.dpad_down;
            case Left1:
                return g1.dpad_left;
            case Up1:
                return g1.dpad_up;
            case A2:
                return g2.a;
            case B2:
                return g2.b;
            case X2:
                return g2.x;
            case Y2:
                return g2.y;
            case Right2:
                return g2.dpad_right;
            case Down2:
                return g2.dpad_down;
            case Left2:
                return g2.dpad_left;
            case Up2:
                return g2.dpad_up;
        }

        return false;
    }
}
