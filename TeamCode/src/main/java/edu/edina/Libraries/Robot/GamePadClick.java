package edu.edina.Libraries.Robot;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamePadClick {
    private final Gamepad gamepad;

    public GamePadClick(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public boolean dpad_up, dpad_down, dpad_left, dpad_right;
    public boolean a, b, x, y;
    public boolean left_bumper, right_bumper;

    public void read() {
        dpad_up = gamepad.dpad_up && !dpad_up;
        dpad_down = gamepad.dpad_down && !dpad_down;
        dpad_left = gamepad.dpad_left && !dpad_left;
        dpad_right = gamepad.dpad_right && !dpad_right;
        a = gamepad.a && !a;
        b = gamepad.b && !b;
        x = gamepad.x && !x;
        y = gamepad.y && !y;
        left_bumper = gamepad.left_bumper && !left_bumper;
        right_bumper = gamepad.right_bumper && !right_bumper;
    }
}
