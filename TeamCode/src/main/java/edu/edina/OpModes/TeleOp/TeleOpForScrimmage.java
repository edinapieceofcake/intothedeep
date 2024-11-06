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

            driveTrain.update();

            compoundArm.update(telemetry);

            telemetry.addData("Main", "====================");
            telemetry.addData("Run Time", runtime.toString());

            telemetry.update();

        }
    }
}
