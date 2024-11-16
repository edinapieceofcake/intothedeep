package edu.edina.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.SampleColor;

@TeleOp
public class LightTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.y)
                hw.setColor(SampleColor.YELLOW);
            else if (gamepad1.b)
                hw.setColor(SampleColor.RED);
            else if (gamepad1.x)
                hw.setColor(SampleColor.BLUE);
            else
                hw.setColor(SampleColor.NOTHING);

            hw.update();
        }
    }
}