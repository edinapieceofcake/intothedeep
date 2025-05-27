package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.edina.Libraries.Robot.RobotState;

@Autonomous
public class RobotStateTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotState rs = new RobotState(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            rs.update();

            telemetry.addData("left lift", rs.getLeftLiftPos());
            telemetry.addData("right lift", rs.getRightLiftPos());
            telemetry.addData("arm", rs.getArmPos());
            telemetry.addData("ext", rs.getExtensionPos());
            telemetry.update();
        }
    }
}
