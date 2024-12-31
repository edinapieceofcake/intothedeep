package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.LinearMotion.LinearMotionController;
import edu.edina.Libraries.LinearMotion.SpringForce;
import edu.edina.Libraries.LinearMotion.VerticalExtensionMechanism;
@Config
@TeleOp


public class LiftTest extends LinearOpMode {
    public static double k=-1;
    public static double g=-20;
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);
        SpringForce springForce = new SpringForce(k,g);
        VerticalExtensionMechanism vertical = new VerticalExtensionMechanism(hw);
        LinearMotionController verticalController = new LinearMotionController(vertical,springForce);

        double lastPos = 0;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                verticalController.setTarget(14);
                lastPos = verticalController.lastPositionAndVelocity().get(0);
            }
            else if (gamepad1.dpad_down) {
                verticalController.setTarget(0);
                lastPos = verticalController.lastPositionAndVelocity().get(0);
            } else
                verticalController.setTarget(lastPos);

            verticalController.run();
        }
    }
}