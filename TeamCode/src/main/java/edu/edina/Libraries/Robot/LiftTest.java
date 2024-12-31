package edu.edina.Libraries.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.LinearMotion.LinearMotionController;
import edu.edina.Libraries.LinearMotion.VerticalExtensionMechanism;

@TeleOp
public class LiftTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);
        VerticalExtensionMechanism vertical = new VerticalExtensionMechanism(hw);
        LinearMotionController verticalController = new LinearMotionController(vertical);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up)
                verticalController.setTarget(14);
            else if (gamepad1.dpad_down)
                verticalController.setTarget(0);
            else
                verticalController.setTarget(vertical.getPositionAndVelocity(false).get(0));

            verticalController.run();
        }
    }
}