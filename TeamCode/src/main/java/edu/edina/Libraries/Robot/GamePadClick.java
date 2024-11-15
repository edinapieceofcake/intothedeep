package edu.edina.Libraries.Robot;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamePadClick {
    private final Gamepad gamepad;
    private Gamepad prev;

    public GamePadClick(Gamepad gamepad) {
        this.gamepad = gamepad;
        prev = new Gamepad();
    }

    public boolean dpad_up, dpad_down, dpad_left, dpad_right;
    public boolean a, b, x, y;

    public void read() {
        dpad_up = gamepad.dpad_up && !prev.dpad_up;
        dpad_down = gamepad.dpad_down && !prev.dpad_down;
        dpad_left = gamepad.dpad_left && !prev.dpad_left;
        dpad_right = gamepad.dpad_right && !prev.dpad_right;
        a = gamepad.a && !prev.a;
        b = gamepad.b && !prev.b;
        x = gamepad.x && !prev.x;
        y = gamepad.y && !prev.y;

        prev.copy(gamepad);
    }
}
