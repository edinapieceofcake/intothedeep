package edu.edina.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import edu.edina.Libraries.Robot.RobotHardwareChicago;
import edu.edina.Libraries.Robot.Speedometer;

@TeleOp(name = "TeleOp Main \uD83C\uDF82", group = "Main")
public class TeleOpChicago extends LinearOpMode {
    private Gamepad currentGamepad1, previousGamepad1, currentGamepad2, previousGamepad2;
    private int cycleNum;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Speedometer s = new Speedometer(20);

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        RobotHardwareChicago hw = new RobotHardwareChicago(hardwareMap);

        while (opModeInInit()) {
            hw.initUpdate();
        }

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            hw.update(telemetry);
            hw.drive(gamepad1, gamepad2);

            if (currentGamepad2.back) {
                hw.brake();
                continue;
            }
            if (previousGamepad2.back && !currentGamepad2.back) {
                hw.calibrateIMU();
            }
            if (currentGamepad1.a && !previousGamepad1.a) {
                hw.toggleClaw();
            }
            if (currentGamepad1.b && !previousGamepad1.b) {
                hw.wallMode();
            }

            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                hw.highSpecimen();
            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper && currentGamepad1.right_trigger >= 0.7) {
                hw.lowSpecimen();
            }
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                hw.highBasket();
            }
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper && currentGamepad1.right_trigger >= 0.7) {
                hw.lowBasket();
            }
            if (hw.armOverSub()) {
                if (currentGamepad2.right_trigger >= 0.7) {
                    hw.intake();
                }

                if (currentGamepad2.left_bumper && previousGamepad2.left_bumper && currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                    hw.halfSwivel();
                } else if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper && currentGamepad2.right_bumper && previousGamepad2.right_bumper) {
                    hw.halfSwivel();
                } else if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                    hw.perpendicularSwivel();
                } else if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                    hw.horizontalSwivel();
                }
                hw.extend(-gamepad2.right_stick_y);
            }

            if (currentGamepad2.x && !previousGamepad2.x) {
                hw.subMode();
            }

            s.sample(cycleNum);
            cycleNum++;

            telemetry.addData("cycleSpeed", "%.1f", 1000.0 / s.getSpeed());

            telemetry.update();
        }
    }
}
