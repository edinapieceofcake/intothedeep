package edu.edina.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.edina.Libraries.Robot.RobotHardwareChicago;

@Autonomous(name = "Reset Robot", group = "Main")
public class ResetMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardwareChicago hw = new RobotHardwareChicago(hardwareMap);

        waitForStart();

        hw.addAction(hw.getArm().constantPower(-.4));
        hw.addAction(hw.getLift().constantPower(-.2));
        hw.addAction(hw.getExtension().constantPower(-.2));
        hw.addAction(hw.getGrabber().wallMode());

        while (opModeIsActive()) {
            hw.update(telemetry);
            telemetry.update();
        }
    }
}
