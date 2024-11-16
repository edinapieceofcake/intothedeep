package edu.edina.Libraries.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled
public class LiftTest extends LinearOpMode {
    private DcMotor leftLift, rightLift;

    public void runOpMode() {
        leftLift = hardwareMap.get(DcMotor.class, "left_lift_motor");
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightLift = hardwareMap.get(DcMotor.class, "right_lift_motor");
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                leftLift.setPower(1);
                //rightLift.setPower(1);
            } else if (gamepad1.dpad_down) {
                leftLift.setPower(-1);
                //rightLift.setPower(-1);
            } else {
                leftLift.setPower(0.0);
                //rightLift.setPower(0.0);
            }

            telemetry.addData("leftLift", leftLift.getCurrentPosition());
            telemetry.addData("rightLift", rightLift.getCurrentPosition());
            telemetry.update();
        }
    }
}
