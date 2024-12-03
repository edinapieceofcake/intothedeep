package edu.edina.Tests.PurePursuit;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.LinearMotion.ThreeAxisDriveMechanism;
import edu.edina.Libraries.Robot.RobotHardware;

@TeleOp
public class TeleOpPurePursuit2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);
        ThreeAxisDriveMechanism driveMechanism = new ThreeAxisDriveMechanism(hw);

        Vector2d[] path = new Vector2d[] {
                new Vector2d(0, 0),
                new Vector2d(20, 20),
        };

        driveMechanism.setPath(path, false);

        waitForStart();

        while (opModeIsActive()) {
            driveMechanism.update();
        }
    }
}