package edu.edina.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import edu.edina.Libraries.LinearMotion.ArmSwingMechanism;
import edu.edina.Libraries.LinearMotion.LinearMotionController;
import edu.edina.Libraries.Robot.RobotHardware;

@TeleOp
public class ClawTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        Servo swivel = hardwareMap.get(Servo.class, "swivel");
        Servo clawTop = hardwareMap.get(Servo.class, "claw_top");
        Servo clawBottom = hardwareMap.get(Servo.class, "claw_bottom");

        RobotHardware hw = new RobotHardware(this);
        ArmSwingMechanism arm = new ArmSwingMechanism(hw);
        LinearMotionController lmc = new LinearMotionController(arm);

        clawBottom.setPosition(0.3917);
        clawTop.setPosition(0.4344);
        swivel.setPosition(0.2244);
        wrist.setPosition(0.1583);

        waitForStart();

        while (opModeIsActive()) {
            //0.1583 start -> 0.8194 end  *wrist*
            //0.2244 wall -> 0.5006 90 -> 0.7728 hanging *swivel*
            //0.9944 open -> 0.4348 closed *claw top*
            //0.0000 open -> 0.3917 closed *claw bottom*

            if (gamepad1.right_bumper) {
                lmc.setTarget(0);
            } else if (gamepad1.left_bumper) {
                lmc.setTarget(-200);
            }

            if (gamepad1.dpad_up) {
                wrist.setPosition(0.8194);
                swivel.setPosition(0.7728);
            } else if (gamepad1.dpad_down) {
                wrist.setPosition(0.1583);
                swivel.setPosition(0.2244);
            }

            if (gamepad1.b) {
                clawTop.setPosition(0.4344);
                clawBottom.setPosition(0.3917);
            } else if (gamepad1.a) {
                clawTop.setPosition(0.9944);
                clawBottom.setPosition(0.0000);
            }

            lmc.run();
        }
    }
}