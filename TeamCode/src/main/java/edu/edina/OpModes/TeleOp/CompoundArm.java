package edu.edina.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CompoundArm {
    private LinearOpMode opMode;
    public CompoundArm(LinearOpMode opMode) throws InterruptedException {
        // Remember the op mode.
        this.opMode = opMode;

        // Get the hardware map.
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Get hardware.

    }

    public void update() throws InterruptedException {
        Gamepad currentGamepad = opMode.gamepad1;

        // Make sure hardware exists.

        // Check gamepad.
            // Set power variables.

        // Turtle mode multipliers.

        // Call set power.
    }
}
