package edu.edina.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.Robot.Light;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.SampleColor;
import edu.edina.Libraries.Robot.SampleSensor;

@TeleOp
//@Disabled
public class LightTest extends LinearOpMode {
    private Light light;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleSensor sampleSensor = new SampleSensor(hardwareMap);
        light = new Light(hardwareMap, sampleSensor);

        waitForStart();

        while (opModeIsActive()) {
            light.update();
        }
    }
}