package edu.edina.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.Libraries.Robot.Arm2;
import edu.edina.Libraries.Robot.RobotState;

@Config
@TeleOp
public class ArmTest extends LinearOpMode {
    public static double POWER_LEVEL = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        Arm2.Mechanism arm = new Arm2.Mechanism(new RobotState(hardwareMap), hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double pos = arm.getPosition(false);

            telemetry.addLine("press up/down to move arm");
            telemetry.addData("position", "%.1f", pos);
            telemetry.update();

            if (pos > 180)
                arm.setPower(0);
            else if (gamepad1.dpad_up)
                arm.setPower(POWER_LEVEL);
            else if (gamepad1.dpad_down)
                arm.setPower(-POWER_LEVEL);
            else
                arm.setPower(0);
        }
    }
}
