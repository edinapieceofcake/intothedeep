package edu.edina.Tests;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import edu.edina.Libraries.LinearMotion.FieldCentricDestination;
import edu.edina.Libraries.LinearMotion.ThreeAxisDriveMechanism;
import edu.edina.Libraries.Robot.DrivingRobotHardware;
import edu.edina.Libraries.Robot.RobotHardware;
import edu.edina.Libraries.Robot.TestRobotHardware;

@TeleOp
public class FieldCentricDriveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double stickDeadzone = 0.02;

        DrivingRobotHardware hw = new TestRobotHardware(this);
        FieldCentricDestination destination = new FieldCentricDestination();
        ThreeAxisDriveMechanism driveMechanism = new ThreeAxisDriveMechanism(hw, destination);

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_x;
            double x = -gamepad1.left_stick_y;
            double h = -gamepad1.right_stick_x;

            RobotLog.ii("FieldCentricDriveTest", "set rel dest");
            driveMechanism.setMaxVelVec(new Vector2d(x, y));
            destination.setRelDest(y, x, h);

            driveMechanism.update();
        }
    }
}
