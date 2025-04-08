package edu.edina.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import edu.edina.Libraries.Robot.RobotHardware;

@TeleOp
@Disabled
public class SubmersibleGrabTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //RobotHardware hw = new RobotHardware(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;

            double axial   = -gamepad1.left_stick_y;
            double lateral   = gamepad1.left_stick_x;

            double leftFrontPower  = -axial;
            double rightFrontPower = axial;
            double leftBackPower   = axial;
            double rightBackPower  = -axial;
            double armPower = lateral;

            armPower = Math.min(armPower, 1);

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left Front Wheel", leftFrontPower);
            telemetry.addData("Right Front Wheel", rightFrontPower);
            telemetry.addData("Left Back Wheel", leftBackPower);
            telemetry.addData("Right Back Wheel", rightBackPower);
            telemetry.addData("Arm", armPower);
            telemetry.update();
        }
    }
}
