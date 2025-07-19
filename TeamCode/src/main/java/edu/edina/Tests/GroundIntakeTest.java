package edu.edina.Tests;

import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.Actions.DriveForwardToDistanceAction;
import edu.edina.Libraries.Actions.SampleAlignAction;
import edu.edina.Libraries.Robot.RobotHardwareChicago;

@TeleOp(name = "Ground Intake Test", group = "Test")
public class GroundIntakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardwareChicago hw = new RobotHardwareChicago(hardwareMap);

        while (opModeInInit()) {
            hw.initUpdate(telemetry);
            telemetry.update();
        }

        hw.addAction(new SequentialAction(
                new SampleAlignAction(hw),
                new DriveForwardToDistanceAction(hw, 3, 1)
        ));

        while (opModeIsActive()) {
            hw.update(telemetry);
            telemetry.update();
        }
    }
}
