package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import edu.edina.Libraries.LinearMotion.FieldCentricDestination;
import edu.edina.Libraries.LinearMotion.ThreeAxisDriveMechanism;
import edu.edina.Libraries.Robot.DrivingRobotHardware;
import edu.edina.Libraries.Robot.TestRobotHardware;

@TeleOp
public class FieldCentricDriveTest extends LinearOpMode {
    public static double STICK_DEADZONE = 0.05;

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

            RobotLog.ii("FieldCentricDriveTest", "set rel dest");

            if (Math.abs(x) > STICK_DEADZONE || Math.abs(y) > STICK_DEADZONE || Math.abs(h) > STICK_DEADZONE)
                destination.setDirection(x, y, h);
            else
                destination.setStopPose(driveMechanism.getEstStopPose());

            driveMechanism.update();
        }
    }
}
