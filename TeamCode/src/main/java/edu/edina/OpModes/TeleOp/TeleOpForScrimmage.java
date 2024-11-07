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

import edu.edina.Libraries.Robot.RobotHardware;

@Config
@TeleOp
public class TeleOpForScrimmage extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        // Get hardware.
        RobotHardware robotHardware = new RobotHardware(this);

        // Wait for the user to lower the lift.
        robotHardware.waitForLiftDown();

        // Wait for the user to lower the arm.
        robotHardware.waitForArmDown();

        // Initialize the robot.
        robotHardware.initializeRobot();

        // If stop is requested...
        if(isStopRequested()) {

            // Exit the method.
            return;

        }

        // Prompt the user to press start.
        robotHardware.log("Waiting for start...");

        waitForStart();

        runtime.reset();

        Gamepad currentGamepad = new Gamepad();
        Gamepad previousGamepad = new Gamepad();

        while (opModeIsActive()) {

            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            if (currentGamepad.a && !previousGamepad.a) {
                robotHardware.compoundArm.toggleClaw();
            }

            if (currentGamepad.x && !previousGamepad.x) {
                robotHardware.compoundArm.toggleWrist();
            }

            if (currentGamepad.left_bumper) {
                robotHardware.compoundArm.retractSlide();
            }
            else if(currentGamepad.right_bumper) {
                robotHardware.compoundArm.extendSlide();
            }
            else {
                robotHardware.compoundArm.stopSlide();
            }

            if (currentGamepad.dpad_down) {
                robotHardware.compoundArm.setArmPosition(ARM_DOWN_POSITION);
            }

            if (currentGamepad.dpad_left) {
                robotHardware.compoundArm.setArmPosition(ARM_WALL_POSITION);
            }

            if (currentGamepad.dpad_up) {
                robotHardware.compoundArm.setArmPosition(ARM_BASKET_POSITION);
            }

            if (currentGamepad.dpad_right) {
                robotHardware.compoundArm.setArmPosition(ARM_CHAMBER_POSITION);
            }

            if (gamepad1.right_trigger > 0.5) {
                robotHardware.compoundArm.setLiftPower(0.3);
            } else if (gamepad1.left_trigger > 0.5) {
                robotHardware.compoundArm.setLiftPower(-0.3);
            } else {
                robotHardware.compoundArm.setLiftPower(0.0);
            }

            robotHardware.driveTrain.update();

            robotHardware.compoundArm.update(telemetry);

            telemetry.addData("Main", "====================");
            telemetry.addData("Run Time", runtime.toString());

            telemetry.update();

        }
    }

}
