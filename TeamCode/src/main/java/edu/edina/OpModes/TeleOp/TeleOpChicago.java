package edu.edina.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import edu.edina.Libraries.Robot.RobotHardwareChicago;

@TeleOp(name = "TeleOp Main \uD83C\uDF82", group = "Main")
public class TeleOpChicago extends LinearOpMode {
    private Gamepad currentGamepad1, previousGamepad1, currentGamepad2, previousGamepad2;

    @Override
    public void runOpMode() throws InterruptedException {
        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        RobotHardwareChicago hw = new RobotHardwareChicago(hardwareMap);

        while (opModeInInit()) {
            hw.initUpdate();
        }

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            hw.update(telemetry);
            hw.drive(gamepad1, gamepad2);
            hw.extend(-gamepad2.right_stick_y);

            if (currentGamepad1.a && !previousGamepad1.a) {
                hw.toggleClaw();
            }
            if (currentGamepad1.b && !previousGamepad1.b) {
                hw.wallMode();
            }
            if (currentGamepad2.x && !previousGamepad2.x) {
                hw.subMode();
            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                hw.highSpecimen();
            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper && currentGamepad1.right_trigger >= 0.7) {
                hw.lowSpecimen();
            }
            if (currentGamepad2.right_trigger >= 0.7) {
                hw.intake();
            }
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                hw.highBasket();
            }
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper && currentGamepad1.right_trigger >= 0.7) {
                hw.lowBasket();
            }
            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                hw.perpendicularSwivel();
            }
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                hw.horizontalSwivel();
            }

            telemetry.update();
        }
    }
}
