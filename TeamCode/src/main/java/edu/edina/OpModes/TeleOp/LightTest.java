package edu.edina.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.Robot.Light;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.SampleColor;

@TeleOp
public class LightTest extends LinearOpMode {
    private Light light;

    @Override
    public void runOpMode() throws InterruptedException {
        light = new Light(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.y)
                setColor(SampleColor.YELLOW);
            else if (gamepad1.b)
                setColor(SampleColor.RED);
            else if (gamepad1.x)
                setColor(SampleColor.BLUE);
            else
                setColor(SampleColor.NOTHING);

            light.update();
        }
    }

    public void setColor(SampleColor sampleColor) {
        light.setSampleColor(sampleColor);
    }
}