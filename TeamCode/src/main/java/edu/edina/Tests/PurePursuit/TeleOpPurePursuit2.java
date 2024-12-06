package edu.edina.Tests.PurePursuit;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.LinearMotion.ThreeAxisDriveMechanism;
import edu.edina.Libraries.Robot.DrivingRobotHardware;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.TestRobotHardware;

@TeleOp
public class TeleOpPurePursuit2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DrivingRobotHardware hw = new TestRobotHardware(this);
        ThreeAxisDriveMechanism driveMechanism = new ThreeAxisDriveMechanism(hw);

        Vector2d[] path = new Vector2d[] {
                new Vector2d(0, 0),
                new Vector2d(10, 20),
                new Vector2d(20, 10)
        };
        Vector2d[] home = new Vector2d[] {
                new Vector2d(0, 0)
        };

        driveMechanism.setPath(path, false);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                driveMechanism.setPath(home, false);
                driveMechanism.setPursuitRadius(2);
            } else if (gamepad1.b) {
                driveMechanism.setPursuitRadius(5);
            }


            driveMechanism.update();
        }
    }
}