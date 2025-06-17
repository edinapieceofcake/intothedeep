package edu.edina.OpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.edina.Libraries.Robot.RobotHardwareChicago;

@Config
@Autonomous
public class SampleStay extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardwareChicago hw = new RobotHardwareChicago(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

        }
    }
}