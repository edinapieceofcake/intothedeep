package edu.edina.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.Robot.RobotHardwareChicago;

@TeleOp(name = "TeleOp Main \uD83C\uDF82", group = "a")
public class TeleOpChicago extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardwareChicago hw = new RobotHardwareChicago(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            hw.update();
            hw.getDrivetrain().update2(gamepad1);
            hw.extend(-gamepad2.left_stick_y);
        }
    }
}
