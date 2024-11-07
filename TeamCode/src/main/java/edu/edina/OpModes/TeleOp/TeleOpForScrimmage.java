package edu.edina.OpModes.TeleOp;

import static edu.edina.OpModes.TeleOp.CompoundArm.ARM_BASKET_POSITION;
import static edu.edina.OpModes.TeleOp.CompoundArm.ARM_CHAMBER_POSITION;
import static edu.edina.OpModes.TeleOp.CompoundArm.ARM_DOWN_POSITION;
import static edu.edina.OpModes.TeleOp.CompoundArm.ARM_WALL_POSITION;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class TeleOpForScrimmage extends LinearOpMode {
    private DriveTrain driveTrain;
    private CompoundArm compoundArm;
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        // Get the robot drive train.
        driveTrain = new DriveTrain(this);

        // Get the robot compound arm
        compoundArm = new CompoundArm(this);

        // Lower the wrist.
        compoundArm.lowerWrist();

        // Open the claw.
        compoundArm.openClaw();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        Gamepad currentGamepad = new Gamepad();
        Gamepad previousGamepad = new Gamepad();

        while (opModeIsActive()) {

            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            if (currentGamepad.a && !previousGamepad.a) {
                compoundArm.toggleClaw();
            }

            if (currentGamepad.x && !previousGamepad.x) {
                compoundArm.toggleWrist();
            }

            if (currentGamepad.left_bumper) {
                compoundArm.retractSlide();
            }
            else if(currentGamepad.right_bumper) {
                compoundArm.extendSlide();
            }
            else {
                compoundArm.stopSlide();
            }

            if (currentGamepad.dpad_down) {
                compoundArm.setArmPosition(ARM_DOWN_POSITION);
            }

            if (currentGamepad.dpad_left) {
                compoundArm.setArmPosition(ARM_WALL_POSITION);
            }

            if (currentGamepad.dpad_up) {
                compoundArm.setArmPosition(ARM_BASKET_POSITION);
            }

            if (currentGamepad.dpad_right) {
                compoundArm.setArmPosition(ARM_CHAMBER_POSITION);
            }

            if (gamepad1.right_trigger > 0.5) {
                compoundArm.setLiftPower(0.3);
            } else if (gamepad1.left_trigger > 0.5) {
                compoundArm.setLiftPower(-0.3);
            } else {
                compoundArm.setLiftPower(0.0);
            }

            driveTrain.update();

            compoundArm.update(telemetry);

            telemetry.addData("Main", "====================");
            telemetry.addData("Run Time", runtime.toString());

            telemetry.update();

        }
    }
}
