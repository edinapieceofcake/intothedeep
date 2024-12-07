package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.LinearMotion.Destination;
import edu.edina.Libraries.LinearMotion.FieldCentricDestination;
import edu.edina.Libraries.LinearMotion.ThreeAxisDriveMechanism;
import edu.edina.Libraries.Robot.DrivingRobotHardware;
import edu.edina.Libraries.Robot.TestRobotHardware;

@TeleOp
public class FieldCentricDriveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DrivingRobotHardware hw = new TestRobotHardware(this);
        FieldCentricDestination destination = new FieldCentricDestination();
        ThreeAxisDriveMechanism driveMechanism = new ThreeAxisDriveMechanism(hw, destination);

        waitForStart();

        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double h = -gamepad1.right_stick_x;

            destination.setRelDest(x, y, h);

            driveMechanism.update();
        }
    }
}
