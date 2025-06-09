package edu.edina.Tests;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;

import edu.edina.Libraries.Robot.ActionList;
import edu.edina.Libraries.Robot.Arm2;
import edu.edina.Libraries.Robot.Extension;
import edu.edina.Libraries.Robot.Grabber;
import edu.edina.Libraries.Robot.RobotState;

@TeleOp(name = "Arm+Extension Test", group = "Test")
public class ArmExtensionTuningTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotState rs = new RobotState(hardwareMap);
        Arm2 arm = new Arm2(rs, hardwareMap);
        Extension ext = new Extension(rs, hardwareMap);
        Grabber grabber = new Grabber(rs, hardwareMap);

        double[] armPositions = new double[]{0, 45, 90, 135, 180};
        int armPosIndex = 0;

        double[] extPositions = new double[]{0, 10};
        int extPosIndex = 0;

        waitForStart();

        ActionList runningActions = new ActionList();
        runningActions.add(grabber.straightWrist());

        while (opModeIsActive()) {
            rs.update(telemetry);

            int nextArmPosIndex;
            if (gamepad1.dpad_right) {
                if (gamepad1.dpad_up) nextArmPosIndex = 1;
                else if (gamepad1.dpad_down) nextArmPosIndex = 7;
                else nextArmPosIndex = 0;
            } else if (gamepad1.dpad_left) {
                if (gamepad1.dpad_up) nextArmPosIndex = 3;
                else if (gamepad1.dpad_down) nextArmPosIndex = 5;
                else nextArmPosIndex = 4;
            } else if (gamepad1.dpad_up) {
                nextArmPosIndex = 2;
            } else if (gamepad1.dpad_down) {
                nextArmPosIndex = 6;
            } else {
                nextArmPosIndex = armPosIndex;
            }

            if (nextArmPosIndex >= armPositions.length)
                nextArmPosIndex = armPositions.length - 1;

            int nextExtPosIndex;
            if (gamepad1.a)
                nextExtPosIndex = 0;
            else if (gamepad1.b)
                nextExtPosIndex = 1;
            else
                nextExtPosIndex = extPosIndex;

            if (nextArmPosIndex != armPosIndex || nextExtPosIndex != extPosIndex) {
                armPosIndex = nextArmPosIndex;
                extPosIndex = nextExtPosIndex;

                runningActions.clear();
                runningActions.add(arm.moveArmWithPid(armPositions[armPosIndex]));
                runningActions.add(ext.moveExtensionWithPid(extPositions[extPosIndex]));
            }

            runningActions.run(new TelemetryPacket());

            telemetry.addData("arm target (use dpad)", "%.0f", armPositions[armPosIndex]);
            telemetry.addData("extension target (a/b)", "%.0f", extPositions[extPosIndex]);
            telemetry.update();
        }
    }
}
